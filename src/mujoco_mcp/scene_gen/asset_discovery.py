#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Dynamic Asset Discovery System

Automatically discovers available objects and robots from:
- assets_db.json (primitives, composites, predefined objects)
- mujoco_menagerie (robot models)

Enables scaling to thousands of assets without manual prompt engineering.
"""

import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Set
import os

logger = logging.getLogger("mujoco_mcp.scene_gen.asset_discovery")


class AssetCatalog:
    """
    Discovers and indexes all available assets for scene generation.

    Provides semantic search and filtering capabilities to dynamically
    construct LLM prompts with only relevant assets.
    """

    def __init__(self, assets_db_path: Optional[str] = None, menagerie_path: Optional[str] = None):
        """
        Initialize asset catalog.

        Args:
            assets_db_path: Path to assets_db.json (auto-detected if None)
            menagerie_path: Path to menagerie directory (from env if None)
        """
        self.assets_db_path = assets_db_path or self._get_default_assets_db()
        self.menagerie_path = (
            menagerie_path or os.getenv("MUJOCO_MENAGERIE_PATH") or self._auto_detect_menagerie()
        )

        # Asset catalogs
        self.primitives: Dict[str, Dict] = {}
        self.composites: Dict[str, Dict] = {}
        self.predefined_objects: Dict[str, Dict] = {}
        self.robots: Dict[str, Dict] = {}

        # Keyword index for semantic search
        self.keyword_index: Dict[str, Set[str]] = {}

        # Discover all assets
        self._discover_assets()
        self._build_keyword_index()

        logger.info(
            f"Asset catalog initialized: "
            f"{len(self.primitives)} primitives, "
            f"{len(self.composites)} composites, "
            f"{len(self.predefined_objects)} predefined objects, "
            f"{len(self.robots)} robots"
        )

    def _get_default_assets_db(self) -> str:
        """Get default path to assets_db.json."""
        return str(Path(__file__).parent / "assets_db.json")

    def _auto_detect_menagerie(self) -> Optional[str]:
        """Auto-detect menagerie path from common locations."""
        # Check sibling directory
        potential_paths = [
            Path(__file__).parent.parent.parent.parent.parent / "mujoco_menagerie",
            Path.home() / "mujoco_menagerie",
            Path("/usr/local/mujoco_menagerie"),
        ]

        for path in potential_paths:
            if path.exists() and path.is_dir():
                logger.info(f"Auto-detected menagerie at: {path}")
                return str(path)

        logger.warning("Could not auto-detect menagerie path")
        return None

    def _discover_assets(self):
        """Discover all available assets from database and menagerie."""
        # Load from assets_db.json
        try:
            with open(self.assets_db_path) as f:
                db = json.load(f)

            for asset_id, asset_data in db.get("assets", {}).items():
                asset_type = asset_data.get("type", "object")

                if asset_id.startswith("primitive:"):
                    self.primitives[asset_id] = asset_data
                elif asset_id.startswith("composite:"):
                    self.composites[asset_id] = asset_data
                elif asset_type == "robot":
                    self.robots[asset_id] = asset_data
                else:
                    self.predefined_objects[asset_id] = asset_data

            logger.info(f"Loaded {len(db.get('assets', {}))} assets from database")
        except Exception as e:
            logger.warning(f"Failed to load assets database: {e}")

        # Discover robots from menagerie
        if self.menagerie_path:
            self._discover_menagerie_robots()

    def _discover_menagerie_robots(self):
        """Discover available robots from mujoco_menagerie."""
        menagerie_dir = Path(self.menagerie_path)
        if not menagerie_dir.exists():
            logger.warning(f"Menagerie path does not exist: {self.menagerie_path}")
            return

        # Known robot directories and their canonical names
        robot_mapping = {
            "franka_emika_panda": {
                "name": "franka_panda",
                "file": "panda.xml",
                "aliases": ["Franka Panda", "Panda", "Franka Emika"],
            },
            "kinova_gen3": {
                "name": "kinova_gen3",
                "file": "gen3.xml",
                "aliases": ["Kinova Gen3", "Gen3"],
            },
            "universal_robots_ur5e": {
                "name": "ur5e",
                "file": "ur5e.xml",
                "aliases": ["UR5e", "Universal Robots UR5e"],
            },
            "universal_robots_ur10e": {
                "name": "ur10e",
                "file": "ur10e.xml",
                "aliases": ["UR10e", "Universal Robots UR10e"],
            },
            "rethink_robotics_sawyer": {
                "name": "sawyer",
                "file": "sawyer.xml",
                "aliases": ["Sawyer", "Rethink Sawyer"],
            },
            "aloha": {"name": "aloha", "file": "scene.xml", "aliases": ["ALOHA", "ALOHA Robot"]},
            "google_robot": {
                "name": "google_robot",
                "file": "scene.xml",
                "aliases": ["Google Robot", "RT-1"],
            },
            "unitree_h1": {
                "name": "unitree_h1",
                "file": "scene.xml",
                "aliases": ["Unitree H1", "H1 Humanoid"],
            },
            "unitree_g1": {
                "name": "unitree_g1",
                "file": "scene.xml",
                "aliases": ["Unitree G1", "G1 Humanoid"],
            },
            "unitree_go2": {
                "name": "unitree_go2",
                "file": "go2.xml",
                "aliases": ["Unitree Go2", "Go2 Quadruped"],
            },
            "boston_dynamics_spot": {
                "name": "spot",
                "file": "scene.xml",
                "aliases": ["Spot", "Boston Dynamics Spot"],
            },
        }

        for menagerie_dir_name, robot_info in robot_mapping.items():
            robot_dir = menagerie_dir / menagerie_dir_name
            robot_file = robot_dir / robot_info["file"]

            if robot_file.exists():
                robot_id = robot_info["name"]
                if robot_id not in self.robots:
                    self.robots[robot_id] = {
                        "type": "robot",
                        "description": f"{robot_info['aliases'][0]} robotic system",
                        "menagerie_dir": menagerie_dir_name,
                        "robot_file": robot_info["file"],
                        "aliases": robot_info["aliases"],
                        "xml_template": f"<!-- Loaded from menagerie: {menagerie_dir_name} -->",
                    }
                    logger.debug(f"Discovered robot: {robot_id} from {menagerie_dir_name}")

        logger.info(f"Discovered {len(self.robots)} robots from menagerie")

    def _build_keyword_index(self):
        """Build keyword index for semantic search."""
        # Index primitives
        for asset_id, data in self.primitives.items():
            keywords = self._extract_keywords(asset_id, data)
            for kw in keywords:
                self.keyword_index.setdefault(kw, set()).add(asset_id)

        # Index composites
        for asset_id, data in self.composites.items():
            keywords = self._extract_keywords(asset_id, data)
            for kw in keywords:
                self.keyword_index.setdefault(kw, set()).add(asset_id)

        # Index predefined objects
        for asset_id, data in self.predefined_objects.items():
            keywords = self._extract_keywords(asset_id, data)
            for kw in keywords:
                self.keyword_index.setdefault(kw, set()).add(asset_id)

        # Index robots
        for robot_id, data in self.robots.items():
            keywords = self._extract_keywords(robot_id, data)
            # Add aliases
            for alias in data.get("aliases", []):
                for word in alias.lower().split():
                    self.keyword_index.setdefault(word, set()).add(robot_id)
            for kw in keywords:
                self.keyword_index.setdefault(kw, set()).add(robot_id)

    def _extract_keywords(self, asset_id: str, data: Dict) -> Set[str]:
        """Extract searchable keywords from asset data."""
        keywords = set()

        # Add ID components
        for part in asset_id.replace(":", "_").replace("_", " ").split():
            keywords.add(part.lower())

        # Add description words
        desc = data.get("description", "")
        for word in desc.lower().split():
            if len(word) > 3:  # Skip short words
                keywords.add(word)

        # Add type
        asset_type = data.get("type", "")
        if asset_type:
            keywords.add(asset_type.lower())

        return keywords

    def search_assets(self, query: str, max_results: int = 20) -> Dict[str, List[str]]:
        """
        Search for relevant assets based on query string.

        Args:
            query: Search query (e.g., user's natural language prompt)
            max_results: Maximum assets to return per category

        Returns:
            Dict with keys: primitives, composites, objects, robots
        """
        query_words = set(query.lower().split())

        # Score assets by keyword matches
        scores: Dict[str, int] = {}

        for word in query_words:
            # Exact matches
            if word in self.keyword_index:
                for asset_id in self.keyword_index[word]:
                    scores[asset_id] = scores.get(asset_id, 0) + 2

            # Partial matches
            for kw, asset_ids in self.keyword_index.items():
                if word in kw or kw in word:
                    for asset_id in asset_ids:
                        scores[asset_id] = scores.get(asset_id, 0) + 1

        # Sort by score
        sorted_assets = sorted(scores.items(), key=lambda x: x[1], reverse=True)

        # Categorize results
        results = {
            "primitives": [],
            "composites": [],
            "objects": [],
            "robots": [],
        }

        for asset_id, score in sorted_assets:
            if asset_id in self.primitives:
                if len(results["primitives"]) < max_results:
                    results["primitives"].append(asset_id)
            elif asset_id in self.composites:
                if len(results["composites"]) < max_results:
                    results["composites"].append(asset_id)
            elif asset_id in self.robots:
                if len(results["robots"]) < max_results:
                    results["robots"].append(asset_id)
            elif asset_id in self.predefined_objects:
                if len(results["objects"]) < max_results:
                    results["objects"].append(asset_id)

        # Always include all primitives and composites (they're fundamental)
        if not results["primitives"]:
            results["primitives"] = list(self.primitives.keys())[:5]
        if not results["composites"]:
            results["composites"] = list(self.composites.keys())[:3]

        return results

    def get_robot_mapping(self) -> Dict[str, tuple]:
        """
        Get robot model mapping for SceneXMLBuilder.

        Returns:
            Dict mapping scene_gen robot names to (menagerie_dir, robot_file) tuples
        """
        mapping = {}
        for robot_id, data in self.robots.items():
            if "menagerie_dir" in data and "robot_file" in data:
                mapping[robot_id] = (data["menagerie_dir"], data["robot_file"])
        return mapping

    def format_assets_for_prompt(
        self, relevant_assets: Optional[Dict[str, List[str]]] = None, compact: bool = True
    ) -> str:
        """
        Format assets for inclusion in LLM prompt.

        Args:
            relevant_assets: Pre-filtered relevant assets (from search_assets)
            compact: Use compact format (one line per asset)

        Returns:
            Formatted string for LLM prompt
        """
        if relevant_assets is None:
            # Return all assets (fallback)
            relevant_assets = {
                "primitives": list(self.primitives.keys()),
                "composites": list(self.composites.keys()),
                "objects": list(self.predefined_objects.keys()),
                "robots": list(self.robots.keys()),
            }

        lines = []

        # Format primitives
        if relevant_assets.get("primitives"):
            lines.append("## Available Primitive Types:")
            for asset_id in relevant_assets["primitives"]:
                data = self.primitives.get(asset_id, {})
                required_dims = ", ".join(data.get("required_dimensions", []))
                desc = data.get("description", "")
                lines.append(f"- **{asset_id}** - requires `{required_dims}` - {desc}")

        # Format composites
        if relevant_assets.get("composites"):
            lines.append("\n## Available Composite Types (containers):")
            for asset_id in relevant_assets["composites"]:
                data = self.composites.get(asset_id, {})
                required_dims = ", ".join(data.get("required_dimensions", []))
                desc = data.get("description", "")
                lines.append(f"- **{asset_id}** - requires `{required_dims}` - {desc}")

        # Format predefined objects
        if relevant_assets.get("objects"):
            lines.append("\n## Available Predefined Objects:")
            for asset_id in relevant_assets["objects"]:
                data = self.predefined_objects.get(asset_id, {})
                desc = data.get("description", "")
                lines.append(f"- {asset_id}: {desc}")

        # Format robots
        if relevant_assets.get("robots"):
            lines.append("\n## Available Robots:")
            for robot_id in relevant_assets["robots"]:
                data = self.robots.get(robot_id, {})
                desc = data.get("description", "")
                aliases = ", ".join(data.get("aliases", [])[:3])
                lines.append(f"- **{robot_id}**: {desc}")
                if aliases:
                    lines.append(f"  Also known as: {aliases}")

        return "\n".join(lines)


if __name__ == "__main__":
    # Test asset discovery
    logging.basicConfig(level=logging.INFO)
    catalog = AssetCatalog()

    print("\n=== Asset Catalog Summary ===")
    print(f"Primitives: {len(catalog.primitives)}")
    print(f"Composites: {len(catalog.composites)}")
    print(f"Predefined Objects: {len(catalog.predefined_objects)}")
    print(f"Robots: {len(catalog.robots)}")

    print("\n=== Available Robots ===")
    for robot_id in catalog.robots.keys():
        print(f"  - {robot_id}")

    print("\n=== Search Test: 'kinova robot' ===")
    results = catalog.search_assets("kinova robot arm manipulation")
    print(f"Found robots: {results['robots']}")

    print("\n=== Formatted Prompt (Compact) ===")
    formatted = catalog.format_assets_for_prompt(results, compact=True)
    print(formatted)
