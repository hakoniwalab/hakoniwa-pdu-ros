import argparse

from hakoniwa_pdu_ros.env_setup import configure_import_paths
from hakoniwa_pdu_ros.bridge_node import run


def main() -> None:
    configure_import_paths()
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args = parser.parse_args()
    run(args.config)
