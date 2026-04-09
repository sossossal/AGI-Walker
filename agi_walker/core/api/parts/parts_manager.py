from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class PartSpec:
    id: str
    category: str
    name: str
    weight_kg: float
    cost_usd: float
    specs: Dict


class PartsManager:
    """Loads the lightweight bundled parts catalog used by tests and demos."""

    def __init__(self, library_path: str | Path | None = None):
        if library_path is None:
            library_path = Path(__file__).with_name("parts_library.json")
        self.library_path = Path(library_path)
        self.parts_db: Dict[str, PartSpec] = {}
        self._load_library()

    def _load_library(self) -> None:
        if not self.library_path.exists():
            raise FileNotFoundError(f"Parts library not found at: {self.library_path}")

        raw_db = json.loads(self.library_path.read_text(encoding="utf-8"))
        self.parts_db.clear()

        for category, items in raw_db.items():
            if not isinstance(items, dict):
                continue
            for part_id, specs in items.items():
                if not isinstance(specs, dict):
                    continue
                self.parts_db[part_id] = PartSpec(
                    id=part_id,
                    category=category,
                    name=specs.get("name", part_id),
                    weight_kg=float(specs.get("weight_kg", 0.0)),
                    cost_usd=float(specs.get("cost_usd", 0.0)),
                    specs=dict(specs),
                )

    def get_part(self, part_id: str) -> Optional[PartSpec]:
        return self.parts_db.get(part_id)

    def list_parts(self, category: str | None = None) -> List[PartSpec]:
        if category is None:
            return list(self.parts_db.values())
        return [part for part in self.parts_db.values() if part.category == category]

    def calculate_bom(self, part_ids: List[str]) -> Dict:
        total_weight = 0.0
        total_cost = 0.0
        details = []
        unknown_parts = []

        for part_id in part_ids:
            part = self.get_part(part_id)
            if part is None:
                unknown_parts.append(part_id)
                continue

            total_weight += part.weight_kg
            total_cost += part.cost_usd
            details.append(
                {
                    "id": part.id,
                    "name": part.name,
                    "category": part.category,
                    "weight_kg": part.weight_kg,
                    "cost_usd": part.cost_usd,
                }
            )

        return {
            "total_weight_kg": total_weight,
            "total_cost_usd": total_cost,
            "part_count": len(details),
            "details": details,
            "unknown_parts": unknown_parts,
        }
