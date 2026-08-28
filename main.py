"""Project entry point."""
import argparse

from robomaster_mission.gui import HybridMazeGUI
from robomaster_mission.mission import legacy_main, robot


def main() -> None:
    parser = argparse.ArgumentParser(
        description="RoboMaster fixed-grid pickup/drop mission"
    )
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="run the original graph DFS without GUI",
    )
    args = parser.parse_args()
    if args.legacy:
        if robot is None:
            raise SystemExit("RoboMaster SDK is not installed")
        legacy_main()
        return
    HybridMazeGUI().run()


if __name__ == "__main__":
    main()
