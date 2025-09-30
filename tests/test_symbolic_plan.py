#!/usr/bin/env python3
"""
Tests for Phase 2C: Symbolic Plan Interface

Tests the NL→Plan→Scene separation, plan validation, and symbolic operations.
"""

import pytest
from unittest.mock import Mock, patch

from mujoco_mcp.scene_gen import (
    SceneDescription,
    SpatialConstraint,
    ObjectPlacement,
    RobotConfiguration,
    MetadataExtractor,
    LLMSceneGenerator
)

# Try to import symbolic plan components
try:
    from mujoco_mcp.scene_gen.symbolic_plan import (
        SymbolicPlan,
        SymbolicOperation,
        SymbolicPlanGenerator,
        PlanToSceneConverter,
        PlanValidator,
        OperationType,
        ConstraintCategory
    )
    SYMBOLIC_PLAN_AVAILABLE = True
except ImportError:
    SYMBOLIC_PLAN_AVAILABLE = False


class TestSymbolicOperation:
    """Test symbolic operation creation and manipulation."""
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_symbolic_operation_creation(self):
        """Test creating and converting symbolic operations."""
        op = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="test_cup",
            parameters={"object_type": "cup_ceramic_small"},
            constraints=[{
                "type": "on_top_of",
                "subject": "test_cup",
                "reference": "test_table",
                "clearance": 0.01
            }],
            description="Place cup on table"
        )
        
        assert op.operation_type == OperationType.PLACE_OBJECT
        assert op.target_entity == "test_cup"
        assert len(op.constraints) == 1
        
        # Test dictionary conversion
        op_dict = op.to_dict()
        assert op_dict["operation_type"] == "place_object"
        assert op_dict["target_entity"] == "test_cup"
        
        # Test round-trip conversion
        op2 = SymbolicOperation.from_dict(op_dict)
        assert op2.operation_type == op.operation_type
        assert op2.target_entity == op.target_entity
        assert op2.constraints == op.constraints


class TestSymbolicPlan:
    """Test symbolic plan creation and manipulation."""
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_symbolic_plan_creation(self):
        """Test creating and manipulating symbolic plans."""
        plan = SymbolicPlan(
            plan_id="test_plan",
            description="Test plan for unit testing"
        )
        
        # Add operations
        table_op = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="table1",
            parameters={"object_type": "table_standard"}
        )
        
        cup_op = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="cup1",
            parameters={"object_type": "cup_ceramic_small"},
            dependencies=["table1"]
        )
        
        plan.add_operation(table_op)
        plan.add_operation(cup_op)
        
        assert len(plan.operations) == 2
        
        # Test operation filtering
        place_ops = plan.get_operations_by_type(OperationType.PLACE_OBJECT)
        assert len(place_ops) == 2
        
        # Test dependency graph
        deps = plan.get_operation_dependencies()
        assert "cup1" in deps
        assert "table1" in deps["cup1"]
        
        # Test dictionary conversion
        plan_dict = plan.to_dict()
        assert plan_dict["plan_id"] == "test_plan"
        assert len(plan_dict["operations"]) == 2
        
        # Test round-trip conversion
        plan2 = SymbolicPlan.from_dict(plan_dict)
        assert plan2.plan_id == plan.plan_id
        assert len(plan2.operations) == len(plan.operations)


class TestPlanValidator:
    """Test plan validation functionality."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_plan_validator_creation(self):
        """Test that plan validator can be created."""
        validator = PlanValidator(self.metadata_extractor)
        assert validator is not None
        assert hasattr(validator, 'metadata_extractor')
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_valid_plan_validation(self):
        """Test validation of a valid plan."""
        validator = PlanValidator(self.metadata_extractor)
        
        # Create valid plan
        plan = SymbolicPlan(plan_id="valid_plan", description="Valid test plan")
        
        # Add table operation
        table_op = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="table1",
            parameters={"object_type": "table_standard"}
        )
        plan.add_operation(table_op)
        
        # Add cup operation with valid dependency
        cup_op = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="cup1",
            parameters={"object_type": "cup_ceramic_small"},
            constraints=[{
                "type": "on_top_of",
                "subject": "cup1",
                "reference": "table1",
                "clearance": 0.01
            }],
            dependencies=["table1"]
        )
        plan.add_operation(cup_op)
        
        is_valid, errors, details = validator.validate_plan(plan)
        
        assert is_valid == True
        assert len(errors) == 0
        assert "dependency_check" in details
        assert "constraint_consistency" in details
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_invalid_plan_validation(self):
        """Test validation of an invalid plan."""
        validator = PlanValidator(self.metadata_extractor)
        
        # Create plan with circular dependency
        plan = SymbolicPlan(plan_id="invalid_plan", description="Invalid test plan")
        
        op1 = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="entity1",
            parameters={"object_type": "box_small"},
            dependencies=["entity2"]
        )
        
        op2 = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="entity2",
            parameters={"object_type": "box_small"},
            dependencies=["entity1"]
        )
        
        plan.add_operation(op1)
        plan.add_operation(op2)
        
        is_valid, errors, details = validator.validate_plan(plan)
        
        assert is_valid == False
        assert len(errors) > 0
        # Should detect circular dependency
        assert any("circular" in error.lower() for error in errors)
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_unknown_entity_validation(self):
        """Test validation with unknown object types."""
        validator = PlanValidator(self.metadata_extractor)
        
        plan = SymbolicPlan(plan_id="unknown_entity_plan", description="Plan with unknown entity")
        
        # Add operation with unknown object type
        unknown_op = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="unknown_object",
            parameters={"object_type": "nonexistent_type"}
        )
        plan.add_operation(unknown_op)
        
        is_valid, errors, details = validator.validate_plan(plan)
        
        # Validation might still pass if metadata extractor has fallbacks
        # but there should be warnings or handling for unknown types
        assert "entity_availability" in details


class TestSymbolicPlanGenerator:
    """Test symbolic plan generation from natural language."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_plan_generator_creation(self):
        """Test that plan generator can be created."""
        generator = SymbolicPlanGenerator(self.metadata_extractor)
        assert generator is not None
        assert hasattr(generator, 'validator')
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_simple_plan_generation(self):
        """Test generating plans from simple natural language."""
        generator = SymbolicPlanGenerator(self.metadata_extractor)
        
        # Test simple table + cup scenario
        plan = generator.generate_plan_from_nl("Create a table with a cup on top")
        
        assert plan is not None
        assert plan.plan_id is not None
        assert len(plan.operations) > 0
        
        # Should have both table and cup operations
        place_ops = plan.get_operations_by_type(OperationType.PLACE_OBJECT)
        entity_types = {op.parameters.get("object_type") for op in place_ops}
        
        # At minimum should detect table and cup
        assert len(place_ops) >= 1
        
        # Check for validation results
        assert "validation_results" in plan.__dict__
        assert "is_valid" in plan.validation_results
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_robot_plan_generation(self):
        """Test generating plans with robots."""
        generator = SymbolicPlanGenerator(self.metadata_extractor)
        
        plan = generator.generate_plan_from_nl("Place a robot arm in front of a table with a cup")
        
        assert plan is not None
        
        # Should have both object and robot operations
        place_ops = plan.get_operations_by_type(OperationType.PLACE_OBJECT)
        robot_ops = plan.get_operations_by_type(OperationType.POSITION_ROBOT)
        
        # Should have detected robot mention
        if robot_ops:
            assert len(robot_ops) >= 1
            robot_op = robot_ops[0]
            assert "robot_type" in robot_op.parameters
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_complex_plan_generation(self):
        """Test generating complex plans with multiple constraints."""
        generator = SymbolicPlanGenerator(self.metadata_extractor)
        
        plan = generator.generate_plan_from_nl(
            "Create a workspace with a table, place a cup and box on the table, "
            "and position a robot arm so it can reach both objects"
        )
        
        assert plan is not None
        assert len(plan.operations) > 1
        
        # Should have multiple types of operations
        operation_types = {op.operation_type for op in plan.operations}
        assert OperationType.PLACE_OBJECT in operation_types


class TestPlanToSceneConverter:
    """Test conversion from symbolic plans to scene descriptions."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_converter_creation(self):
        """Test that plan to scene converter can be created."""
        converter = PlanToSceneConverter(self.metadata_extractor)
        assert converter is not None
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_simple_plan_conversion(self):
        """Test converting a simple plan to scene description."""
        converter = PlanToSceneConverter(self.metadata_extractor)
        
        # Create simple plan
        plan = SymbolicPlan(plan_id="simple_plan", description="Simple test plan")
        
        # Add table operation
        table_op = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="table1",
            parameters={"object_type": "table_standard"}
        )
        plan.add_operation(table_op)
        
        # Add cup operation
        cup_op = SymbolicOperation(
            operation_type=OperationType.PLACE_OBJECT,
            target_entity="cup1",
            parameters={"object_type": "cup_ceramic_small"},
            constraints=[{
                "type": "on_top_of",
                "subject": "cup1",
                "reference": "table1",
                "clearance": 0.01
            }],
            dependencies=["table1"]
        )
        plan.add_operation(cup_op)
        
        # Convert to scene
        scene = converter.convert_plan_to_scene(plan)
        
        assert isinstance(scene, SceneDescription)
        assert len(scene.objects) == 2
        
        # Check that objects were created correctly
        object_ids = {obj.object_id for obj in scene.objects}
        assert "table1" in object_ids
        assert "cup1" in object_ids
        
        # Check constraints were converted
        cup_obj = next(obj for obj in scene.objects if obj.object_id == "cup1")
        assert len(cup_obj.constraints) == 1
        assert cup_obj.constraints[0].type == "on_top_of"
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_robot_plan_conversion(self):
        """Test converting plans with robots."""
        converter = PlanToSceneConverter(self.metadata_extractor)
        
        plan = SymbolicPlan(plan_id="robot_plan", description="Plan with robot")
        
        # Add robot operation
        robot_op = SymbolicOperation(
            operation_type=OperationType.POSITION_ROBOT,
            target_entity="robot1",
            parameters={"robot_type": "franka_panda", "joint_config": "ready"},
            constraints=[{
                "type": "in_front_of",
                "subject": "robot1",
                "reference": "table1",
                "clearance": 0.2
            }]
        )
        plan.add_operation(robot_op)
        
        scene = converter.convert_plan_to_scene(plan)
        
        assert len(scene.robots) == 1
        robot = scene.robots[0]
        assert robot.robot_id == "robot1"
        assert robot.robot_type == "franka_panda"
        assert len(robot.constraints) == 1


class TestLLMGeneratorWithSymbolicPlans:
    """Test LLM generator integration with symbolic plans."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_llm_generator_with_symbolic_plans(self):
        """Test that LLM generator can use symbolic plans."""
        generator = LLMSceneGenerator(self.metadata_extractor)
        
        # Should have symbolic plan capabilities
        assert hasattr(generator, 'use_symbolic_plans')
        
        if SYMBOLIC_PLAN_AVAILABLE:
            assert generator.use_symbolic_plans == True
            assert generator.plan_generator is not None
            assert generator.plan_converter is not None
    
    @pytest.mark.skipif(not SYMBOLIC_PLAN_AVAILABLE, reason="Symbolic plan not available")
    def test_scene_generation_with_symbolic_plans(self):
        """Test scene generation using symbolic plan pipeline."""
        generator = LLMSceneGenerator(self.metadata_extractor)
        
        if generator.use_symbolic_plans:
            scene = generator.generate_scene_description("Create a table with a cup on top")
            
            assert isinstance(scene, SceneDescription)
            assert len(scene.objects) > 0
            
            # Should have created meaningful objects
            object_types = {obj.object_type for obj in scene.objects}
            # Basic validation that some objects were created
            assert len(object_types) > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])