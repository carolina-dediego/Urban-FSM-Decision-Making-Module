import argparse
import copy
import csv
import json
import re
from pathlib import Path


def extract_real_id(filename: str) -> int | None:
    """
    Acepta nombres tipo:
    - results_00.json
    - results_0.json
    - results_ruta_00.json
    - route_00.json
    """
    patterns = [
        r"^results_(\d{1,2})\.json$",
        r"^results_ruta_(\d{1,2})\.json$",
        r"^route_(\d{1,2})\.json$",
    ]

    for pattern in patterns:
        match = re.match(pattern, filename)
        if match:
            return int(match.group(1))

    return None


def get_records(data: dict) -> list:
    return data.get("_checkpoint", {}).get("records", [])


def count_infractions(infractions: dict) -> int:
    if not isinstance(infractions, dict):
        return 0

    total = 0
    for value in infractions.values():
        if isinstance(value, list):
            total += len(value)

    return total


def is_strict_success(record: dict) -> bool:
    status = str(record.get("status", ""))
    infractions = record.get("infractions", {})
    score_route = record.get("scores", {}).get("score_route", 0)
    score_penalty = record.get("scores", {}).get("score_penalty", 0)

    return (
        status == "Perfect"
        and float(score_route) >= 100
        and float(score_penalty) >= 1.0
        and count_infractions(infractions) == 0
    )


def main():
    parser = argparse.ArgumentParser(
        description="Merge individual Bench2Drive JSON results into one 44-route results JSON."
    )

    parser.add_argument(
        "--input",
        default="json_individuales",
        help="Folder containing individual JSON files: results_00.json ... results_43.json",
    )

    parser.add_argument(
        "--output",
        default="merged_44_results.json",
        help="Output merged JSON file.",
    )

    parser.add_argument(
        "--expected",
        type=int,
        default=44,
        help="Expected number of routes.",
    )

    parser.add_argument(
        "--report",
        default="merge_report.csv",
        help="CSV report with merged route summary.",
    )

    args = parser.parse_args()

    input_folder = Path(args.input)
    output_file = Path(args.output)
    report_file = Path(args.report)

    if not input_folder.exists():
        raise FileNotFoundError(f"Input folder not found: {input_folder}")

    merged_records = []
    report_rows = []
    used_ids = set()

    json_files = sorted(input_folder.glob("*.json"))

    if not json_files:
        raise RuntimeError(f"No JSON files found in folder: {input_folder}")

    for json_path in json_files:
        real_id = extract_real_id(json_path.name)

        if real_id is None:
            print(f"WARNING: skipped file with invalid name: {json_path.name}")
            continue

        if real_id < 0 or real_id >= args.expected:
            print(f"WARNING: skipped file with id outside 0-{args.expected - 1}: {json_path.name}")
            continue

        if real_id in used_ids:
            raise RuntimeError(f"Duplicated route id {real_id} in file: {json_path.name}")

        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        records = get_records(data)

        if len(records) == 0:
            print(f"WARNING: no records found in {json_path.name}")
            continue

        if len(records) > 1:
            print(f"WARNING: {json_path.name} has {len(records)} records. Using the first one only.")

        record = copy.deepcopy(records[0])

        # Reasignación del ID real dentro del conjunto de 44 rutas
        record["index"] = real_id
        record["route_id"] = f"RouteScenario_{real_id}_rep0"
        record["_new_id"] = str(real_id)
        record["_success"] = is_strict_success(record)

        merged_records.append(record)
        used_ids.add(real_id)

        scores = record.get("scores", {})
        meta = record.get("meta", {})

        report_rows.append({
            "new_id": real_id,
            "file": json_path.name,
            "route_id": record.get("route_id", ""),
            "status": record.get("status", ""),
            "score_route": scores.get("score_route", ""),
            "score_penalty": scores.get("score_penalty", ""),
            "score_composed": scores.get("score_composed", ""),
            "num_infractions": record.get("num_infractions", ""),
            "success_strict": record.get("_success", False),
            "duration_game": meta.get("duration_game", ""),
            "duration_system": meta.get("duration_system", ""),
        })

    merged_records.sort(key=lambda r: int(r["index"]))
    report_rows.sort(key=lambda r: int(r["new_id"]))

    found_ids = sorted(used_ids)
    expected_ids = set(range(args.expected))
    missing_ids = sorted(expected_ids - used_ids)

    merged_data = {
        "_checkpoint": {
            "global_record": {},
            "progress": [len(merged_records), args.expected],
            "records": merged_records,
        },
        "entry_status": "Finished" if len(merged_records) == args.expected else "Partial",
        "eligible": len(merged_records) == args.expected,
        "labels": [],
        "values": [],
    }

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(merged_data, f, indent=2)

    with open(report_file, "w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "new_id",
            "file",
            "route_id",
            "status",
            "score_route",
            "score_penalty",
            "score_composed",
            "num_infractions",
            "success_strict",
            "duration_game",
            "duration_system",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(report_rows)

    print()
    print("Merge finished.")
    print(f"Output JSON: {output_file}")
    print(f"Report CSV:  {report_file}")
    print(f"Routes merged: {len(merged_records)}/{args.expected}")
    print(f"Found ids: {found_ids}")

    if missing_ids:
        print(f"WARNING: missing ids: {missing_ids}")
    else:
        print("All expected route ids are present.")


if __name__ == "__main__":
    main()