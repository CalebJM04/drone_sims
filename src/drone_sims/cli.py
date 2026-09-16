from __future__ import annotations

import argparse
import json
from pathlib import Path

from .campaign import run_campaign
from .scenarios import CAMPAIGN_SCENARIOS, SCENARIOS, build
from .verification import run_full_verification
from .provenance import collect_metadata


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description="Drone mesh simulation foundation")
    commands = root.add_subparsers(dest="command", required=True)
    scenario = commands.add_parser("scenario", help="run one end-to-end scenario")
    scenario.add_argument("name", choices=SCENARIOS)
    scenario.add_argument("--seed", type=int, default=1)
    scenario.add_argument("--output", type=Path)
    scenario.add_argument("--trace-interval", type=float, default=0.1)
    campaign = commands.add_parser("campaign", help="run repeated randomized scenarios")
    campaign.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(CAMPAIGN_SCENARIOS))
    campaign.add_argument("--seeds", type=int, default=20)
    campaign.add_argument("--workers", type=int, default=1)
    campaign.add_argument("--output", type=Path)
    verify = commands.add_parser("verify", help="run every locally available verification layer")
    verify.add_argument("--cases", type=int, default=10_000)
    verify.add_argument("--campaign-seeds", type=int, default=20)
    verify.add_argument("--workers", type=int, default=1)
    verify.add_argument("--endurance-seeds", type=int, default=3)
    verify.add_argument("--output", type=Path, default=Path("results/full"))
    return root


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.command == "scenario":
        simulation = build(args.name, args.seed)
        simulation.config.trace_interval = args.trace_interval
        report = simulation.run()
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps({
                    "metadata": collect_metadata(
                        "drone-sims scenario",
                        seeds=[args.seed],
                        parameters={
                            "scenario": args.name,
                            "seed": args.seed,
                            "trace_interval": args.trace_interval,
                        },
                    ),
                    "summary": report,
                    "events": simulation.events,
                    "trace": simulation.trace,
                }, indent=2) + "\n",
                encoding="utf-8",
            )
    elif args.command == "campaign":
        report = run_campaign(args.scenarios, args.seeds, args.workers)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    elif args.command == "verify":
        run_full_verification(
            args.cases, args.campaign_seeds, args.workers, args.output, args.endurance_seeds,
        )
        print("Verification finished.")
        print(f"Results: {args.output / 'results.txt'}")
        return 0
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
