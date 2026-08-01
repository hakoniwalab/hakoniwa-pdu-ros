from __future__ import annotations

import argparse
import sys

from hakoniwa_pdu_ros.env_setup import configure_import_paths
from hakoniwa_pdu_ros.service_config_generator import generate_service_configs


def main() -> None:
    configure_import_paths()
    parser = argparse.ArgumentParser(
        description=(
            "Generate Hakoniwa PDU-RPC server/client configs from a "
            "ROS Service Server Binding"
        )
    )
    parser.add_argument("--config", required=True, help="Service Binding JSON path")
    parser.add_argument(
        "--output-dir",
        help="Output directory (default: build/generated/service/<binding-id>)",
    )
    parser.add_argument(
        "--offset-dir",
        help="Hakoniwa offset root (fallback: HAKO_BINARY_PATH)",
    )
    args = parser.parse_args()

    try:
        generated = generate_service_configs(
            args.config,
            output_dir=args.output_dir,
            offset_dir=args.offset_dir,
        )
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1) from None

    print(f"Server config: {generated.server_config}")
    print(f"Client config: {generated.client_config}")


if __name__ == "__main__":
    main()
