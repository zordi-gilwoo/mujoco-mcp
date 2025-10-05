#!/usr/bin/env python3
"""
MuJoCo Menagerie Model Loader
Handles downloading and loading of MuJoCo Menagerie models with include resolution
"""

import os
import urllib.request
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Optional, Tuple, List
import logging

logger = logging.getLogger(__name__)


class MenagerieLoader:
    """Load MuJoCo Menagerie models with automatic include resolution"""

    BASE_URL = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main"

    def __init__(self, cache_dir: Optional[str] = None, menagerie_root: Optional[str] = None):
        self.cache_dir = (
            Path(cache_dir) if cache_dir else Path(tempfile.gettempdir()) / "mujoco_menagerie"
        )
        self.cache_dir.mkdir(exist_ok=True)

        env_root = menagerie_root or os.environ.get("MUJOCO_MENAGERIE_PATH")
        self.menagerie_root: Optional[Path] = None
        if env_root:
            candidate = Path(env_root).expanduser()
            if candidate.exists():
                self.menagerie_root = candidate
            else:
                logger.warning(
                    "MUJOCO_MENAGERIE_PATH set to %s, but directory does not exist", candidate
                )

        if self.menagerie_root is None:
            base_path = Path(__file__).resolve()
            repo_candidate = base_path.parents[2] / "mujoco_menagerie"
            if repo_candidate.exists():
                self.menagerie_root = repo_candidate
            else:
                sibling_candidate = base_path.parents[3] / "mujoco_menagerie"
                if sibling_candidate.exists():
                    self.menagerie_root = sibling_candidate

    def download_file(self, model_name: str, file_path: str) -> str:
        """Download a file from the Menagerie repository"""
        url = f"{self.BASE_URL}/{model_name}/{file_path}"

        # Check cache first
        cache_file = self.cache_dir / model_name / file_path
        if cache_file.exists():
            return cache_file.read_text()

        try:
            with urllib.request.urlopen(url, timeout=10) as response:
                if response.getcode() == 200:
                    content = response.read().decode("utf-8")

                    # Save to cache
                    cache_file.parent.mkdir(parents=True, exist_ok=True)
                    cache_file.write_text(content)

                    return content
                else:
                    raise Exception(f"HTTP {response.getcode()}")
        except Exception as e:
            raise Exception(f"Failed to download {url}: {e}")

    def resolve_includes(
        self, xml_content: str, model_name: str, visited: Optional[set] = None
    ) -> str:
        """Resolve XML include directives recursively"""
        if visited is None:
            visited = set()

        try:
            root = ET.fromstring(xml_content)
        except ET.ParseError as e:
            logger.warning(f"XML parse error: {e}")
            return xml_content

        # Find all include elements
        for parent in list(root.iter()):
            children = list(parent)
            for idx, include in enumerate(children):
                if include.tag != "include":
                    continue

                file_attr = include.get("file")
                if not file_attr:
                    continue

                if file_attr in visited:
                    logger.warning(f"Circular include detected: {file_attr}")
                    continue

                include_path = self._ensure_file_path(model_name, file_attr)
                if include_path is None:
                    logger.warning(f"Failed to resolve include {file_attr}: file not found")
                    continue

                try:
                    included_content = include_path.read_text()
                except Exception as exc:
                    logger.warning(f"Failed to read include {file_attr}: {exc}")
                    continue

                nested_visited = visited.copy()
                nested_visited.add(file_attr)
                included_content = self.resolve_includes(
                    included_content, model_name, nested_visited
                )

                try:
                    included_root = ET.fromstring(included_content)
                except ET.ParseError as exc:
                    logger.warning(f"Failed to parse include {file_attr}: {exc}")
                    continue

                # Remove the include placeholder
                parent.remove(include)

                # Merge root attributes when including full <mujoco> documents
                if parent is root and included_root.tag == "mujoco":
                    for key, value in included_root.attrib.items():
                        parent.attrib.setdefault(key, value)

                insert_idx = idx
                for child in list(included_root):
                    parent.insert(insert_idx, child)
                    insert_idx += 1

        # Return modified XML
        return ET.tostring(root, encoding="unicode")

    def get_model_xml(self, model_name: str) -> str:
        """Get complete XML for a Menagerie model with includes resolved"""

        # Try different common file patterns
        possible_files = [f"{model_name}.xml", "scene.xml", f"{model_name}_mjx.xml"]

        for xml_file in possible_files:
            try:
                file_path = self._ensure_file_path(model_name, xml_file)
                if file_path is None:
                    continue

                xml_content = file_path.read_text()

                # Resolve includes
                resolved_xml = self.resolve_includes(xml_content, model_name)

                logger.info(f"Successfully loaded {model_name} from {xml_file}")
                return resolved_xml

            except Exception as e:
                logger.debug(f"Failed to load {model_name} from {xml_file}: {e}")
                continue

        raise Exception(f"Could not load any XML files for model {model_name}")

    def _ensure_file_path(self, model_name: str, file_path: str) -> Optional[Path]:
        """Return a filesystem path for a menagerie asset, downloading if necessary."""

        if self.menagerie_root:
            local_path = self.menagerie_root / model_name / file_path
            if local_path.exists():
                return local_path

        cache_path = self.cache_dir / model_name / file_path
        if cache_path.exists():
            return cache_path

        cache_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            content = self.download_file(model_name, file_path)
        except Exception as exc:
            logger.debug(f"Failed to fetch {model_name}/{file_path}: {exc}")
            return None

        # download_file already wrote the content to cache_path
        return cache_path if content is not None else None

    def get_model_file(self, model_name: str) -> Path:
        """Locate the primary XML file for a menagerie model.

        Prefers the local menagerie checkout when available, falling back to the
        downloader cache. Raises FileNotFoundError if no XML can be located.
        """

        candidates = [
            "scene.xml",
            f"{model_name}.xml",
            f"{model_name}_mjx.xml",
        ]

        for candidate in candidates:
            path = self._ensure_file_path(model_name, candidate)
            if path is not None:
                return path

        raise FileNotFoundError(f"No XML found for menagerie model '{model_name}'.")

    def get_available_models(self) -> Dict[str, List[str]]:
        """Get list of available models by category (cached/hardcoded for performance)"""
        return {
            "arms": [
                "franka_emika_panda",
                "universal_robots_ur5e",
                "kinova_gen3",
                "kinova_jaco2",
                "barrett_wam",
                "ufactory_lite6",
                "ufactory_xarm7",
                "abb_irb1600",
                "fanuc_m20ia",
                "kuka_iiwa_14",
                "rethink_sawyer",
            ],
            "quadrupeds": [
                "unitree_go2",
                "unitree_go1",
                "unitree_a1",
                "boston_dynamics_spot",
                "anybotics_anymal_c",
                "anybotics_anymal_b",
                "google_barkour_v0",
                "mit_mini_cheetah",
            ],
            "humanoids": [
                "unitree_h1",
                "unitree_g1",
                "apptronik_apollo",
                "pal_talos",
                "berkeley_humanoid",
                "nasa_valkyrie",
                "honda_asimo",
                "boston_dynamics_atlas",
                "agility_cassie",
            ],
            "grippers": [
                "robotiq_2f85",
                "robotiq_2f140",
                "shadow_hand",
                "leap_hand",
                "wonik_allegro",
                "barrett_hand",
            ],
            "mobile_manipulators": [
                "google_robot",
                "hello_robot_stretch",
                "clearpath_ridgeback_ur5e",
            ],
            "drones": ["skydio_x2", "crazyflie_2"],
        }

    def validate_model(self, model_name: str) -> Dict[str, any]:
        """Validate that a model can be loaded and return info"""
        try:
            xml_content = self.get_model_xml(model_name)

            # Basic validation
            if not xml_content.strip():
                return {"valid": False, "error": "Empty XML content"}

            # Try to parse XML
            try:
                root = ET.fromstring(xml_content)
                if root.tag != "mujoco":
                    return {
                        "valid": False,
                        "error": "Not a valid MuJoCo XML (root is not 'mujoco')",
                    }
            except ET.ParseError as e:
                return {"valid": False, "error": f"XML parse error: {e}"}

            # Try MuJoCo loading if available
            try:
                import mujoco

                with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as tmp:
                    tmp.write(xml_content)
                    tmp_path = tmp.name

                try:
                    model = mujoco.MjModel.from_xml_path(tmp_path)
                    result = {
                        "valid": True,
                        "n_bodies": model.nbody,
                        "n_joints": model.njnt,
                        "n_actuators": model.nu,
                        "xml_size": len(xml_content),
                    }
                finally:
                    os.unlink(tmp_path)

                return result

            except ImportError:
                # MuJoCo not available, just return basic validation
                return {
                    "valid": True,
                    "xml_size": len(xml_content),
                    "note": "MuJoCo validation skipped (not installed)",
                }

        except Exception as e:
            return {"valid": False, "error": str(e)}

    def create_scene_xml(self, model_name: str, scene_name: Optional[str] = None) -> str:
        """Create a complete scene XML for a Menagerie model"""
        model_path = self.get_model_file(model_name)
        model_xml = model_path.read_text()

        # If the model XML is already a complete scene, return it
        if "<worldbody>" in model_xml and "<mujoco>" in model_xml:
            return model_xml

        # Otherwise, wrap it in a scene template
        scene_template = f"""
        <mujoco model="{scene_name or model_name}_scene">
          <compiler angle="radian" meshdir="." texturedir="."/>
          <option timestep="0.002" integrator="RK4"/>
          
          <default>
            <joint damping="0.1"/>
            <geom contype="1" conaffinity="1"/>
          </default>
          
          <asset>
            <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
            <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance="0"/>
          </asset>
          
          <worldbody>
            <geom name="floor" size="0 0 0.05" type="plane" material="grid"/>
            <light name="light" pos="0 0 1"/>
            
            {model_xml}
          </worldbody>
        </mujoco>
        """

        return scene_template.strip()
