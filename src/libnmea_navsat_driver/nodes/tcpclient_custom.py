#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

import socket
import sys
import binascii
import re

import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver


# Regex opzionale: accetta solo frasi con checksum finale "*HH"
# Se il tuo ricevitore invia NMEA senza checksum, disabilita questa verifica.
NMEA_WITH_CHECKSUM_RE = re.compile(r'^\$.*\*[0-9A-Fa-f]{2}$')


def main(args=None):
    rclpy.init(args=args)
    driver = Ros2NMEADriver()

    try:
        gnss_ip = driver.declare_parameter('ip', '192.168.1.110').value
        gnss_port = driver.declare_parameter('port', 9904).value
        buffer_size = driver.declare_parameter('buffer_size', 4096).value

        # Parametri aggiuntivi (compatibili: se non li setti, default sensati)
        strict_nmea_checksum = driver.declare_parameter('strict_nmea_checksum', True).value
        log_non_ascii_samples = driver.declare_parameter('log_non_ascii_samples', True).value
        non_ascii_log_every = driver.declare_parameter('non_ascii_log_every', 200).value

    except KeyError as e:
        driver.get_logger().error("Parameter %s not found" % e)
        sys.exit(1)

    frame_id = driver.get_frame_id()

    driver.get_logger().info(
        "Using gnss sensor with ip {} and port {} (strict_nmea_checksum={})"
        .format(gnss_ip, gnss_port, strict_nmea_checksum)
    )

    # Connection-loop: connect and keep receiving. If receiving fails, reconnect
    while rclpy.ok():
        try:
            gnss_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            gnss_socket.connect((gnss_ip, gnss_port))
        except socket.error as exc:
            driver.get_logger().error(
                "Caught exception socket.error when setting up socket: %s" % exc
            )
            sys.exit(1)

        # recv-loop: When we're connected, keep receiving stuff until that fails
        partial = b""
        non_ascii_count = 0
        garbage_count = 0

        while rclpy.ok():
            try:
                chunk = gnss_socket.recv(buffer_size)

                if not chunk:
                    driver.get_logger().warn("GNSS socket closed by peer.")
                    gnss_socket.close()
                    break

                partial += chunk

                # Split on LF at bytes-level (TCP delivers bytes, not lines)
                parts = partial.split(b'\n')

                # If last byte is LF, last element after split is b'' (complete)
                if partial.endswith(b'\n'):
                    full_lines = parts[:-1]
                    partial = b""
                else:
                    full_lines = parts[:-1]
                    partial = parts[-1]

                for raw in full_lines:
                    # Handle CRLF
                    raw = raw.rstrip(b'\r')
                    if not raw:
                        continue

                    # Decode ASCII per-line: if it fails, drop the offending line
                    try:
                        data = raw.decode("ascii")
                    except UnicodeDecodeError:
                        non_ascii_count += 1

                        # Log occasional sample in hex for diagnosis (no spam)
                        if log_non_ascii_samples and (
                            non_ascii_count <= 3 or (non_ascii_log_every > 0 and non_ascii_count % non_ascii_log_every == 0)
                        ):
                            sample_hex = binascii.hexlify(raw[:32]).decode("ascii")
                            driver.get_logger().warn(
                                f"Non-ASCII bytes in GNSS stream (count={non_ascii_count}); "
                                f"sample_hex={sample_hex}..."
                            )
                        continue

                    data = data.strip()
                    if not data:
                        continue

                    # Basic NMEA sanity: should start with '$'
                    if not data.startswith("$"):
                        garbage_count += 1
                        continue

                    # Optional strict checksum sanity
                    if strict_nmea_checksum and not NMEA_WITH_CHECKSUM_RE.match(data):
                        garbage_count += 1
                        continue

                    try:
                        if driver.add_sentence(data, frame_id):
                            driver.get_logger().debug("Received sentence: %s" % data)
                        else:
                            driver.get_logger().warn("Error with sentence: %s" % data)
                    except ValueError as e:
                        driver.get_logger().warn(
                            "Value error, likely due to missing fields in the NMEA message. "
                            "Error was: %s. " % e
                        )

            except socket.error as exc:
                driver.get_logger().error(
                    "Caught exception socket.error when receiving: %s" % exc
                )
                gnss_socket.close()
                break

        gnss_socket.close()


if __name__ == "__main__":
    main()
