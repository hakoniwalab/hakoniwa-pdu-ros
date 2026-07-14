import argparse
import sys

from hakoniwa_pdu_ros.env_setup import configure_import_paths
from hakoniwa_pdu_ros.bridge_node import run
from hakoniwa_pdu_ros.zenoh_io import ZenohIoValidationError


def main() -> None:
    configure_import_paths()
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args = parser.parse_args()
    try:
        run(args.config)
    except ZenohIoValidationError as err:
        print(err, file=sys.stderr)
        raise SystemExit(1) from None
