from __future__ import annotations

import argparse

from hakoniwa_pdu_ros.zenoh_io import print_zenoh_io, write_zenoh_io


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("binding_config")
    parser.add_argument("--comm", help="Zenoh comm JSON to update.")
    parser.add_argument("--write", action="store_true", help="Update --comm in place.")
    args = parser.parse_args()

    if args.write:
        if not args.comm:
            parser.error("--write requires --comm")
        write_zenoh_io(args.binding_config, args.comm)
        return

    print_zenoh_io(args.binding_config)


if __name__ == "__main__":
    main()
