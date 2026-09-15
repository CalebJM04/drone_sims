from __future__ import annotations

import argparse
import json
from pathlib import Path

from .campaign import run_campaign
from .companion_runtime import run_service
from .integrations import readiness
from .mesh_demo import load_requirements, run_mesh_demo
from .mesh_visualization import LiveMeshDashboard, build_mesh_dashboard
from .network_matrix import run_network_matrix
from .scenarios import CAMPAIGN_SCENARIOS, SCENARIOS, build
from .verification import run_full_verification
from .visualization import DEFAULT_TRACE_SCENARIOS, build_dashboard
from .provenance import collect_metadata


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description="LoRa UAV simulation and verification suite")
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
    commands.add_parser("readiness", help="report companion-computer and SITL tooling")
    companion = commands.add_parser("companion", help="run the Raspberry Pi/PX4 companion service")
    companion.add_argument("--config", type=Path, required=True)
    companion.add_argument(
        "--enable-control", action="store_true",
        help="stream velocity setpoints (never arms or changes PX4 flight mode)",
    )
    network = commands.add_parser("network-matrix", help="compare exact LoRa PHY/MAC/routing choices")
    network.add_argument("--seeds", type=int, default=3)
    network.add_argument("--output", type=Path, default=Path("results/full/network_matrix.json"))
    visualize = commands.add_parser("visualize", help="build a self-contained HTML results dashboard")
    visualize.add_argument("--results", type=Path, default=Path("results/full"))
    visualize.add_argument("--output", type=Path, default=Path("results/full/dashboard.html"))
    visualize.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(DEFAULT_TRACE_SCENARIOS))
    visualize.add_argument("--seed", type=int, default=7)
    mesh = commands.add_parser(
        "mesh-demo", help="run 4-8 real companion services over a simulated LoRa mesh"
    )
    mesh.add_argument("--nodes", type=int, choices=range(4, 9), default=6)
    mesh.add_argument("--duration", type=float, default=15.0)
    mesh.add_argument("--seed", type=int, default=31)
    mesh.add_argument("--realtime", action="store_true", help="pace virtual time to wall time")
    mesh.add_argument("--serve", type=int, metavar="PORT", help="serve a live dashboard on localhost")
    mesh.add_argument("--output", type=Path, default=Path("results/mesh/demo.json"))
    mesh.add_argument("--dashboard", type=Path, default=Path("results/mesh/dashboard.html"))
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
    elif args.command == "companion":
        return run_service(args.config, enable_control=args.enable_control)
    elif args.command == "network-matrix":
        report = run_network_matrix(args.seeds)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    elif args.command == "visualize":
        output = build_dashboard(args.results, args.output, scenarios=args.scenarios, seed=args.seed)
        report = {"dashboard": str(output), "scenarios": args.scenarios, "seed": args.seed}
    elif args.command == "mesh-demo":
        live = LiveMeshDashboard(args.serve) if args.serve is not None else None
        last_printed = -1

        def update(state: dict[str, object]) -> None:
            nonlocal last_printed
            if live is not None:
                live.update(state)
            second = int(float(state["time_s"]))
            if second != last_printed:
                last_printed = second
                print(json.dumps({
                    "time_s": state["time_s"],
                    "delivery_ratio": state["delivery_ratio"],
                    "route_1_to_2": state["route_1_to_2"],
                    "active_alerts": len(state["alerts"]),
                }, sort_keys=True), flush=True)

        if live is not None:
            live.start()
            print(f"Live dashboard: {live.address}", flush=True)
        try:
            report = run_mesh_demo(
                nodes=args.nodes,
                duration_s=args.duration,
                seed=args.seed,
                realtime=args.realtime,
                requirements=load_requirements(),
                update=update,
            )
        finally:
            if live is not None:
                live.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        dashboard = build_mesh_dashboard(report, args.dashboard)
        print(json.dumps({"status": report["status"], "summary": report["summary"], "dashboard": str(dashboard)}, indent=2))
        return 0 if report["status"] == "PASS" else 1
    else:
        report = readiness()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
