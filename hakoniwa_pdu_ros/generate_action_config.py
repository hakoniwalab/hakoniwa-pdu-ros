from __future__ import annotations

import argparse
from pathlib import Path

from hakoniwa_pdu_ros.action_config_generator import generate_action_configs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate Hakoniwa PDU-RPC configuration from a ROS Action Binding"
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    try:
        generated = generate_action_configs(
            args.config, output_dir=args.output_dir
        )
    except ValueError as error:
        parser.error(str(error))
    print(generated.manifest)
    for path in generated.generated_files:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
