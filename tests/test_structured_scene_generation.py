#!/usr/bin/env python3
"""
Tests for Structured Scene Generation

Comprehensive tests for the scene generation pipeline including:
- Schema validation
- Metadata extraction
- Constraint solving
- XML generation
- MCP tool integration
"""

import json
import pytest
import numpy as np
from unittest.mock import Mock, patch
from pathlib import Path

from mujoco_mcp.scene_gen import (
    SceneDescription,
    SpatialConstraint,
    ObjectPlacement,
    RobotConfiguration,
    MetadataExtractor,
    ConstraintSolver,
    SceneXMLBuilder,
    LLMSceneGenerator,
    Pose
)
from mujoco_mcp.scene_gen.metadata_extractor import AssetMetadata


class TestSceneSchema:
    """Test Pydantic scene schema models."""
    
    def test_spatial_constraint_validation(self):
        """Test spatial constraint validation."""
        # Valid constraint
        constraint = SpatialConstraint(
            type="on_top_of",
            subject="cup_1",
            reference="table_1",
            clearance=0.01
        )
        assert constraint.type == "on_top_of"
        assert constraint.clearance == 0.01
        
        # Invalid constraint type
        with pytest.raises(ValueError, match="Constraint type must be one of"):
            SpatialConstraint(
                type="invalid_type",
                subject="cup_1", 
                reference="table_1"
            )
    
    def test_object_placement_validation(self):
        """Test object placement validation."""
        # Valid object placement
        obj = ObjectPlacement(
            object_id="table_1",
            object_type="table_standard",
            constraints=[]
        )
        assert obj.object_id == "table_1"
        assert obj.object_type == "table_standard"
        
        # Empty object ID
        with pytest.raises(ValueError, match="Object ID cannot be empty"):
            ObjectPlacement(
                object_id="",
                object_type="table_standard"
            )
    
    def test_scene_description_validation(self):
        """Test scene description validation including constraint references."""
        # Valid scene
        scene_dict = {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup_1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup_1",
                            "reference": "table_1",
                            "clearance": 0.01
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        assert len(scene.objects) == 2
        assert scene.get_all_entity_ids() == ["table_1", "cup_1"]
        
        # Invalid reference
        invalid_scene_dict = {
            "objects": [
                {
                    "object_id": "cup_1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup_1",
                            "reference": "nonexistent_table",
                            "clearance": 0.01
                        }
                    ]
                }
            ]
        }
        
        with pytest.raises(ValueError, match="constraint referencing unknown entity"):
            SceneDescription(**invalid_scene_dict)


class TestMetadataExtractor:
    """Test metadata extraction functionality."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.extractor = MetadataExtractor()
    
    def test_static_metadata_loading(self):
        """Test loading of static asset metadata."""
        # Should load from assets_db.json
        metadata = self.extractor.get_metadata("table_standard")
        assert metadata is not None
        assert metadata.asset_id == "table_standard"
        assert metadata.asset_type == "table"
        
        # Check dimensions
        dims = metadata.get_dimensions()
        assert "width" in dims
        assert "height" in dims
        
        # Check bounding box
        bbox_min, bbox_max = metadata.get_bounding_box()
        assert len(bbox_min) == 3
        assert len(bbox_max) == 3
    
    def test_xml_parsing_extraction(self):
        """Test metadata extraction from XML string."""
        xml_str = """
        <mujoco>
            <worldbody>
                <body>
                    <geom type="box" size="0.5 0.3 0.1" pos="0 0 0.1"/>
                </body>
            </worldbody>
        </mujoco>
        """
        
        metadata = self.extractor.extract_from_xml(xml_str, "test_object")
        
        assert metadata.asset_id == "test_object"
        dims = metadata.get_dimensions()
        assert dims["width"] > 0
        assert dims["height"] > 0
    
    def test_fallback_metadata(self):
        """Test fallback metadata generation."""
        metadata = self.extractor._create_fallback_metadata("unknown_asset")
        
        assert metadata.asset_id == "unknown_asset"
        assert metadata.asset_type == "fallback"
        dims = metadata.get_dimensions()
        assert dims["width"] == 0.1
        assert dims["height"] == 0.1


class TestConstraintSolver:
    """Test constraint solving functionality."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.extractor = MetadataExtractor()
        self.solver = ConstraintSolver(self.extractor)
    
    def test_simple_constraint_solving(self):
        """Test solving a simple scene with table and cup."""
        scene_dict = {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup_1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup_1",
                            "reference": "table_1",
                            "clearance": 0.01
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        poses = self.solver.solve(scene)
        
        assert len(poses) == 2
        assert "table_1" in poses
        assert "cup_1" in poses
        
        # Cup should be above table
        table_pos = poses["table_1"].position
        cup_pos = poses["cup_1"].position
        assert cup_pos[2] > table_pos[2]  # Cup Z > Table Z
    
    def test_collision_constraint(self):
        """Test no_collision constraint solving."""
        scene_dict = {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "box_1",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_1",
                            "reference": "table_1",
                            "clearance": 0.001
                        }
                    ]
                },
                {
                    "object_id": "box_2",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_2",
                            "reference": "table_1", 
                            "clearance": 0.001
                        },
                        {
                            "type": "no_collision",
                            "subject": "box_2",
                            "reference": "box_1",
                            "clearance": 0.05
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        poses = self.solver.solve(scene)
        
        assert len(poses) == 3
        
        # Boxes should be separated
        box1_pos = poses["box_1"].position
        box2_pos = poses["box_2"].position
        distance = np.linalg.norm(box1_pos - box2_pos)
        assert distance > 0.05  # Should respect clearance
    
    def test_robot_constraint_solving(self):
        """Test solving constraints with robots."""
        scene_dict = {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard",
                    "constraints": []
                }
            ],
            "robots": [
                {
                    "robot_id": "franka_1",
                    "robot_type": "franka_panda",
                    "joint_config": "ready",
                    "constraints": [
                        {
                            "type": "in_front_of",
                            "subject": "franka_1",
                            "reference": "table_1",
                            "clearance": 0.2
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        poses = self.solver.solve(scene)
        
        assert len(poses) == 2
        assert "franka_1" in poses
        
        # Robot should be in front of table (positive X direction)
        table_pos = poses["table_1"].position
        robot_pos = poses["franka_1"].position
        assert robot_pos[0] > table_pos[0]


class TestSceneXMLBuilder:
    """Test XML scene building functionality."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.extractor = MetadataExtractor()
        self.builder = SceneXMLBuilder(self.extractor)
    
    def test_xml_generation(self):
        """Test generation of valid XML from scene and poses."""
        scene_dict = {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup_1",
                    "object_type": "cup_ceramic_small",
                    "constraints": []
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        poses = {
            "table_1": Pose(
                position=np.array([0.0, 0.0, 0.0]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0])
            ),
            "cup_1": Pose(
                position=np.array([0.0, 0.0, 0.8]),
                orientation=np.array([0.0, 0.0, 0.0, 1.0])
            )
        }
        
        xml_str = self.builder.build_scene(scene, poses, "test_scene")
        
        # Check that XML is valid
        assert self.builder.validate_xml(xml_str)
        
        # Check for expected elements
        assert "<mujoco" in xml_str
        assert "<worldbody>" in xml_str
        assert "table_1" in xml_str
        assert "cup_1" in xml_str
        assert "floor" in xml_str
    
    def test_xml_validation(self):
        """Test XML validation functionality."""
        valid_xml = "<mujoco><worldbody></worldbody></mujoco>"
        assert self.builder.validate_xml(valid_xml)
        
        invalid_xml = "<mujoco><worldbody></mujoco>"  # Missing closing tag
        assert not self.builder.validate_xml(invalid_xml)


class TestLLMSceneGenerator:
    """Test LLM scene generation functionality."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.extractor = MetadataExtractor()
        self.generator = LLMSceneGenerator(self.extractor)
    
    def test_canned_example_generation(self):
        """Test generation of canned example scenes."""
        # Test different prompt types - enhanced version uses keyword matching
        prompts_and_expected = [
            ("Create a table with a cup and robot", 3),  # table + cup + robot
            ("I need a table and box collision test scene", 2),  # table + box
            ("Make a simple table scene", 1),  # table only
            ("Put a cup on the table", 2)  # table + cup
        ]
        
        for prompt, expected_entities in prompts_and_expected:
            scene_dict = self.generator.generate_or_stub(prompt)
            
            # Should return valid JSON that can create SceneDescription
            scene = SceneDescription(**scene_dict)
            entity_ids = scene.get_all_entity_ids()
            assert len(entity_ids) == expected_entities, f"Expected {expected_entities} entities for '{prompt}', got {len(entity_ids)}: {entity_ids}"
    
    def test_llm_prompt_building(self):
        """Test LLM prompt construction."""
        prompt = self.generator.build_llm_prompt("Create a simple scene")
        
        assert "JSON scene description" in prompt
        assert "table_standard" in prompt
        assert "franka_panda" in prompt
    
    @patch.dict('os.environ', {'STRUCTURED_SCENE_LLM': 'enabled'})
    def test_llm_integration_placeholder(self):
        """Test that LLM integration raises NotImplementedError when enabled."""
        generator = LLMSceneGenerator(self.extractor)
        
        with pytest.raises(NotImplementedError, match="Real LLM integration not yet implemented"):
            generator.generate_scene_description("test prompt")


class TestIntegration:
    """Integration tests for the complete scene generation pipeline."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.extractor = MetadataExtractor()
        self.solver = ConstraintSolver(self.extractor)
        self.builder = SceneXMLBuilder(self.extractor)
        self.llm_generator = LLMSceneGenerator(self.extractor)
    
    def test_end_to_end_pipeline(self):
        """Test the complete pipeline from natural language to XML."""
        # Generate scene from natural language
        scene_dict = self.llm_generator.generate_or_stub("table with cup and robot")
        scene = SceneDescription(**scene_dict)
        
        # Solve constraints
        poses = self.solver.solve(scene)
        
        # Build XML
        xml_str = self.builder.build_scene(scene, poses, "integration_test")
        
        # Validate result
        assert self.builder.validate_xml(xml_str)
        assert len(xml_str) > 1000  # Should be substantial XML
        assert "<mujoco" in xml_str
        assert len(poses) == len(scene.get_all_entity_ids())
    
    def test_json_input_pipeline(self):
        """Test pipeline with JSON scene description input."""
        scene_json = """
        {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup_1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup_1",
                            "reference": "table_1",
                            "clearance": 0.002
                        }
                    ]
                }
            ],
            "robots": [],
            "workspace_bounds": [-1.0, -1.0, 0.0, 1.5, 1.0, 2.0]
        }
        """
        
        # Parse JSON
        scene_dict = json.loads(scene_json)
        scene = SceneDescription(**scene_dict)
        
        # Solve constraints
        poses = self.solver.solve(scene)
        
        # Build XML
        xml_str = self.builder.build_scene(scene, poses, "json_test")
        
        # Validate result
        assert self.builder.validate_xml(xml_str)
        assert "cup_1" in xml_str
        assert "table_1" in xml_str


# Test that would require MuJoCo - skip if not available
class TestMuJoCoIntegration:
    """Tests that require MuJoCo to be installed."""
    
    def setup_method(self):
        """Set up test fixtures."""
        try:
            import mujoco
            self.mujoco_available = True
        except ImportError:
            self.mujoco_available = False
    
    @pytest.mark.skipif(True, reason="MuJoCo integration test placeholder")
    def test_xml_loads_in_mujoco(self):
        """Test that generated XML can be loaded by MuJoCo."""
        pytest.skip("MuJoCo integration test - implement when MuJoCo is available")


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v"])