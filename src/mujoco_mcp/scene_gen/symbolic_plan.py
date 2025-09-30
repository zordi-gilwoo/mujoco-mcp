#!/usr/bin/env python3
"""
Symbolic Plan Interface

Implements Phase 2C of the enhanced scene generation system:
1. Explicit NL→Plan→Scene separation 
2. Plan validation and constraint auditing
3. Symbolic operation primitives

This addresses the limitation in PR #12 where natural language goes directly
to structured scene without an intermediate auditable plan representation.
"""

import logging
import json
from typing import Dict, List, Tuple, Optional, Union, Any
from dataclasses import dataclass, field
from enum import Enum
from pydantic import BaseModel, Field, field_validator

from .scene_schema import SceneDescription, SpatialConstraint, ObjectPlacement, RobotConfiguration
from .metadata_extractor import MetadataExtractor

logger = logging.getLogger("mujoco_mcp.scene_gen.symbolic_plan")


class OperationType(Enum):
    """Types of symbolic operations in a plan."""
    PLACE_OBJECT = "place_object"
    POSITION_ROBOT = "position_robot"
    APPLY_CONSTRAINT = "apply_constraint"
    SET_WORKSPACE = "set_workspace"
    VALIDATE_SAFETY = "validate_safety"


class ConstraintCategory(Enum):
    """Categories of constraints for plan validation."""
    SPATIAL_RELATIONSHIP = "spatial_relationship"  # on_top_of, beside, etc.
    COLLISION_AVOIDANCE = "collision_avoidance"    # no_collision
    REACHABILITY = "reachability"                  # within_reach
    ORIENTATION = "orientation"                     # oriented_towards, aligned_with_axis
    CONTAINMENT = "containment"                    # inside


@dataclass
class SymbolicOperation:
    """
    Represents a single symbolic operation in a plan.
    """
    operation_type: OperationType
    target_entity: str  # Entity being operated on
    parameters: Dict[str, Any] = field(default_factory=dict)
    constraints: List[Dict[str, Any]] = field(default_factory=list)
    priority: int = 0  # Higher numbers = higher priority
    dependencies: List[str] = field(default_factory=list)  # Must complete before this
    description: str = ""
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            "operation_type": self.operation_type.value,
            "target_entity": self.target_entity,
            "parameters": self.parameters,
            "constraints": self.constraints,
            "priority": self.priority,
            "dependencies": self.dependencies,
            "description": self.description
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SymbolicOperation':
        """Create from dictionary representation."""
        return cls(
            operation_type=OperationType(data["operation_type"]),
            target_entity=data["target_entity"],
            parameters=data.get("parameters", {}),
            constraints=data.get("constraints", []),
            priority=data.get("priority", 0),
            dependencies=data.get("dependencies", []),
            description=data.get("description", "")
        )


@dataclass
class SymbolicPlan:
    """
    Represents a complete symbolic plan for scene generation.
    """
    plan_id: str
    description: str
    operations: List[SymbolicOperation] = field(default_factory=list)
    global_constraints: List[Dict[str, Any]] = field(default_factory=list)
    workspace_bounds: Optional[Tuple[float, float, float, float, float, float]] = None
    validation_results: Dict[str, Any] = field(default_factory=dict)
    
    def add_operation(self, operation: SymbolicOperation):
        """Add an operation to the plan."""
        self.operations.append(operation)
    
    def get_operations_by_type(self, operation_type: OperationType) -> List[SymbolicOperation]:
        """Get all operations of a specific type."""
        return [op for op in self.operations if op.operation_type == operation_type]
    
    def get_operation_dependencies(self) -> Dict[str, List[str]]:
        """Get dependency graph for operations."""
        dependencies = {}
        for op in self.operations:
            dependencies[op.target_entity] = op.dependencies
        return dependencies
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            "plan_id": self.plan_id,
            "description": self.description,
            "operations": [op.to_dict() for op in self.operations],
            "global_constraints": self.global_constraints,
            "workspace_bounds": self.workspace_bounds,
            "validation_results": self.validation_results
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SymbolicPlan':
        """Create from dictionary representation."""
        plan = cls(
            plan_id=data["plan_id"],
            description=data["description"],
            global_constraints=data.get("global_constraints", []),
            workspace_bounds=data.get("workspace_bounds"),
            validation_results=data.get("validation_results", {})
        )
        
        for op_data in data.get("operations", []):
            plan.add_operation(SymbolicOperation.from_dict(op_data))
        
        return plan


class PlanValidator:
    """
    Validates symbolic plans for consistency, safety, and feasibility.
    """
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
    
    def validate_plan(self, plan: SymbolicPlan) -> Tuple[bool, List[str], Dict[str, Any]]:
        """
        Comprehensive plan validation.
        
        Returns:
            (is_valid, validation_errors, validation_details)
        """
        errors = []
        details = {
            "dependency_check": None,
            "constraint_consistency": None,
            "entity_availability": None,
            "spatial_feasibility": None,
            "safety_analysis": None
        }
        
        # 1. Check operation dependencies
        dep_valid, dep_errors = self._validate_dependencies(plan)
        details["dependency_check"] = {"valid": dep_valid, "errors": dep_errors}
        errors.extend(dep_errors)
        
        # 2. Check constraint consistency
        const_valid, const_errors = self._validate_constraint_consistency(plan)
        details["constraint_consistency"] = {"valid": const_valid, "errors": const_errors}
        errors.extend(const_errors)
        
        # 3. Check entity availability
        entity_valid, entity_errors = self._validate_entity_availability(plan)
        details["entity_availability"] = {"valid": entity_valid, "errors": entity_errors}
        errors.extend(entity_errors)
        
        # 4. Check spatial feasibility
        spatial_valid, spatial_errors = self._validate_spatial_feasibility(plan)
        details["spatial_feasibility"] = {"valid": spatial_valid, "errors": spatial_errors}
        errors.extend(spatial_errors)
        
        # 5. Safety analysis
        safety_valid, safety_errors = self._validate_safety(plan)
        details["safety_analysis"] = {"valid": safety_valid, "errors": safety_errors}
        errors.extend(safety_errors)
        
        is_valid = len(errors) == 0
        
        logger.info(f"Plan validation completed: {'VALID' if is_valid else 'INVALID'} ({len(errors)} errors)")
        
        return is_valid, errors, details
    
    def _validate_dependencies(self, plan: SymbolicPlan) -> Tuple[bool, List[str]]:
        """Validate operation dependency graph."""
        errors = []
        
        # Check for circular dependencies
        entity_deps = plan.get_operation_dependencies()
        visited = set()
        rec_stack = set()
        
        def has_cycle(entity):
            if entity in rec_stack:
                return True
            if entity in visited:
                return False
            
            visited.add(entity)
            rec_stack.add(entity)
            
            for dep in entity_deps.get(entity, []):
                if has_cycle(dep):
                    return True
            
            rec_stack.remove(entity)
            return False
        
        for entity in entity_deps:
            if entity not in visited:
                if has_cycle(entity):
                    errors.append(f"Circular dependency detected involving entity: {entity}")
        
        # Check that all dependencies exist
        all_entities = {op.target_entity for op in plan.operations}
        for op in plan.operations:
            for dep in op.dependencies:
                if dep not in all_entities:
                    errors.append(f"Operation {op.target_entity} depends on non-existent entity: {dep}")
        
        return len(errors) == 0, errors
    
    def _validate_constraint_consistency(self, plan: SymbolicPlan) -> Tuple[bool, List[str]]:
        """Validate that constraints are consistent and don't conflict."""
        errors = []
        
        # Collect all constraints
        all_constraints = []
        for op in plan.operations:
            for constraint in op.constraints:
                all_constraints.append((op.target_entity, constraint))
        
        # Check for contradictory constraints
        entity_constraints = {}
        for entity, constraint in all_constraints:
            if entity not in entity_constraints:
                entity_constraints[entity] = []
            entity_constraints[entity].append(constraint)
        
        for entity, constraints in entity_constraints.items():
            # Check for conflicting spatial relationships
            spatial_refs = {}
            for constraint in constraints:
                constraint_type = constraint.get("type")
                reference = constraint.get("reference")
                
                if constraint_type in ["on_top_of", "beside", "in_front_of"]:
                    if reference in spatial_refs:
                        prev_type = spatial_refs[reference]
                        if prev_type != constraint_type:
                            errors.append(f"Conflicting spatial constraints for {entity} relative to {reference}: {prev_type} vs {constraint_type}")
                    spatial_refs[reference] = constraint_type
        
        return len(errors) == 0, errors
    
    def _validate_entity_availability(self, plan: SymbolicPlan) -> Tuple[bool, List[str]]:
        """Validate that all referenced entities are available."""
        errors = []
        
        # Check that object types exist in metadata
        for op in plan.operations:
            if op.operation_type == OperationType.PLACE_OBJECT:
                object_type = op.parameters.get("object_type")
                if object_type:
                    metadata = self.metadata_extractor.get_metadata(object_type)
                    if metadata is None:
                        errors.append(f"Unknown object type: {object_type} for entity {op.target_entity}")
            
            elif op.operation_type == OperationType.POSITION_ROBOT:
                robot_type = op.parameters.get("robot_type")
                if robot_type:
                    metadata = self.metadata_extractor.get_metadata(robot_type)
                    if metadata is None:
                        errors.append(f"Unknown robot type: {robot_type} for entity {op.target_entity}")
        
        return len(errors) == 0, errors
    
    def _validate_spatial_feasibility(self, plan: SymbolicPlan) -> Tuple[bool, List[str]]:
        """Validate spatial feasibility of the plan."""
        errors = []
        
        # Basic spatial constraint validation
        place_operations = plan.get_operations_by_type(OperationType.PLACE_OBJECT)
        
        # Check that objects placed "on_top_of" have valid support surfaces
        for op in place_operations:
            for constraint in op.constraints:
                if constraint.get("type") == "on_top_of":
                    reference = constraint.get("reference")
                    
                    # Find the reference entity
                    ref_op = None
                    for other_op in plan.operations:
                        if other_op.target_entity == reference:
                            ref_op = other_op
                            break
                    
                    if ref_op is None:
                        errors.append(f"Object {op.target_entity} placed on_top_of unknown reference: {reference}")
                    else:
                        # Check if reference can support objects
                        ref_type = ref_op.parameters.get("object_type")
                        if ref_type and "table" not in ref_type.lower():
                            # This is a heuristic - could be more sophisticated
                            logger.warning(f"Object {op.target_entity} placed on {reference} which may not be a good support surface")
        
        return len(errors) == 0, errors
    
    def _validate_safety(self, plan: SymbolicPlan) -> Tuple[bool, List[str]]:
        """Validate safety aspects of the plan."""
        errors = []
        warnings = []
        
        # Check for collision avoidance
        place_operations = plan.get_operations_by_type(OperationType.PLACE_OBJECT)
        robot_operations = plan.get_operations_by_type(OperationType.POSITION_ROBOT)
        
        # Ensure robots have adequate clearance from objects
        for robot_op in robot_operations:
            robot_has_clearance = False
            for constraint in robot_op.constraints:
                if constraint.get("type") in ["in_front_of", "beside"]:
                    clearance = constraint.get("clearance", 0)
                    if clearance >= 0.1:  # Minimum 10cm clearance
                        robot_has_clearance = True
                        break
            
            if not robot_has_clearance:
                warnings.append(f"Robot {robot_op.target_entity} may not have adequate clearance from objects")
        
        # Check for explicit collision avoidance between objects
        object_pairs = []
        for i, op1 in enumerate(place_operations):
            for op2 in place_operations[i+1:]:
                object_pairs.append((op1.target_entity, op2.target_entity))
        
        collision_constraints = []
        for op in place_operations:
            for constraint in op.constraints:
                if constraint.get("type") == "no_collision":
                    collision_constraints.append((op.target_entity, constraint.get("reference")))
        
        # This is more of a warning than an error
        missing_collision_checks = []
        for obj1, obj2 in object_pairs:
            if (obj1, obj2) not in collision_constraints and (obj2, obj1) not in collision_constraints:
                missing_collision_checks.append((obj1, obj2))
        
        if missing_collision_checks:
            warnings.append(f"Missing explicit collision avoidance for {len(missing_collision_checks)} object pairs")
        
        logger.debug(f"Safety validation: {len(errors)} errors, {len(warnings)} warnings")
        
        return len(errors) == 0, errors


class SymbolicPlanGenerator:
    """
    Generates symbolic plans from natural language descriptions.
    This provides the explicit NL→Plan separation.
    """
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
        self.validator = PlanValidator(metadata_extractor)
    
    def generate_plan_from_nl(self, natural_language: str, plan_id: Optional[str] = None) -> SymbolicPlan:
        """
        Generate symbolic plan from natural language description.
        This implements the NL→Plan phase of the pipeline.
        """
        if plan_id is None:
            plan_id = f"plan_{hash(natural_language) % 10000:04d}"
        
        plan = SymbolicPlan(
            plan_id=plan_id,
            description=f"Generated from: {natural_language[:100]}..."
        )
        
        # Parse natural language and generate operations
        # For now, use rule-based parsing - this could be enhanced with LLM
        operations = self._parse_natural_language(natural_language)
        
        for operation in operations:
            plan.add_operation(operation)
        
        # Validate the generated plan
        is_valid, errors, details = self.validator.validate_plan(plan)
        plan.validation_results = {
            "is_valid": is_valid,
            "errors": errors,
            "details": details,
            "generated_at": "nl_parsing"
        }
        
        if not is_valid:
            logger.warning(f"Generated plan has validation errors: {errors}")
        else:
            logger.info(f"Generated valid plan with {len(plan.operations)} operations")
        
        return plan
    
    def _parse_natural_language(self, text: str) -> List[SymbolicOperation]:
        """
        Parse natural language into symbolic operations.
        This is a simplified rule-based parser - could be enhanced with NLP/LLM.
        """
        operations = []
        text_lower = text.lower()
        
        # Detect table creation
        if "table" in text_lower:
            operations.append(SymbolicOperation(
                operation_type=OperationType.PLACE_OBJECT,
                target_entity="table1",
                parameters={"object_type": "table_standard"},
                description="Place table as workspace surface"
            ))
        
        # Detect cup placement
        if "cup" in text_lower:
            constraints = []
            if "table" in text_lower or "on" in text_lower:
                constraints.append({
                    "type": "on_top_of",
                    "subject": "cup1",
                    "reference": "table1",
                    "clearance": 0.002
                })
            
            operations.append(SymbolicOperation(
                operation_type=OperationType.PLACE_OBJECT,
                target_entity="cup1",
                parameters={"object_type": "cup_ceramic_small"},
                constraints=constraints,
                dependencies=["table1"] if constraints else [],
                description="Place cup on table"
            ))
        
        # Detect box placement
        if "box" in text_lower:
            constraints = []
            if "table" in text_lower:
                constraints.append({
                    "type": "on_top_of",
                    "subject": "box1",
                    "reference": "table1",
                    "clearance": 0.001
                })
            
            # Add collision avoidance if cup is also mentioned
            if "cup" in text_lower:
                constraints.append({
                    "type": "no_collision",
                    "subject": "box1",
                    "reference": "cup1",
                    "clearance": 0.05
                })
            
            operations.append(SymbolicOperation(
                operation_type=OperationType.PLACE_OBJECT,
                target_entity="box1",
                parameters={"object_type": "box_small"},
                constraints=constraints,
                dependencies=["table1"] if constraints else [],
                description="Place box with collision avoidance"
            ))
        
        # Detect robot placement
        if "robot" in text_lower or "arm" in text_lower:
            constraints = []
            if "front" in text_lower or "table" in text_lower:
                constraints.append({
                    "type": "in_front_of",
                    "subject": "robot1",
                    "reference": "table1",
                    "clearance": 0.2
                })
            
            # Add reachability constraints
            if "cup" in text_lower or "reach" in text_lower:
                constraints.append({
                    "type": "within_reach",
                    "subject": "robot1",
                    "reference": "cup1",
                    "clearance": 0.1
                })
            
            operations.append(SymbolicOperation(
                operation_type=OperationType.POSITION_ROBOT,
                target_entity="robot1",
                parameters={"robot_type": "franka_panda", "joint_config": "ready"},
                constraints=constraints,
                dependencies=["table1"],
                description="Position robot for manipulation tasks"
            ))
        
        return operations


class PlanToSceneConverter:
    """
    Converts symbolic plans to structured scene descriptions.
    This implements the Plan→Scene phase of the pipeline.
    """
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
    
    def convert_plan_to_scene(self, plan: SymbolicPlan) -> SceneDescription:
        """
        Convert a symbolic plan to a structured scene description.
        This implements the Plan→Scene phase of the pipeline.
        """
        objects = []
        robots = []
        
        # Process operations in dependency order
        sorted_operations = self._sort_operations_by_dependencies(plan.operations)
        
        for operation in sorted_operations:
            if operation.operation_type == OperationType.PLACE_OBJECT:
                obj = self._convert_place_object_operation(operation)
                objects.append(obj)
            
            elif operation.operation_type == OperationType.POSITION_ROBOT:
                robot = self._convert_position_robot_operation(operation)
                robots.append(robot)
        
        # Create scene description
        scene = SceneDescription(
            objects=objects,
            robots=robots,
            workspace_bounds=plan.workspace_bounds
        )
        
        logger.info(f"Converted plan {plan.plan_id} to scene with {len(objects)} objects and {len(robots)} robots")
        
        return scene
    
    def _sort_operations_by_dependencies(self, operations: List[SymbolicOperation]) -> List[SymbolicOperation]:
        """Sort operations by their dependencies using topological sort."""
        # Build dependency graph
        deps = {op.target_entity: op.dependencies for op in operations}
        op_map = {op.target_entity: op for op in operations}
        
        # Topological sort
        sorted_entities = []
        visited = set()
        temp_visited = set()
        
        def visit(entity):
            if entity in temp_visited:
                # Circular dependency - handle gracefully
                logger.warning(f"Circular dependency detected involving {entity}")
                return
            if entity in visited:
                return
            
            temp_visited.add(entity)
            for dep in deps.get(entity, []):
                if dep in deps:  # Only visit if it's an entity with operations
                    visit(dep)
            temp_visited.remove(entity)
            visited.add(entity)
            sorted_entities.append(entity)
        
        for entity in deps:
            if entity not in visited:
                visit(entity)
        
        # Return operations in sorted order
        sorted_operations = []
        for entity in sorted_entities:
            if entity in op_map:
                sorted_operations.append(op_map[entity])
        
        return sorted_operations
    
    def _convert_place_object_operation(self, operation: SymbolicOperation) -> ObjectPlacement:
        """Convert a PLACE_OBJECT operation to an ObjectPlacement."""
        # Convert constraints
        spatial_constraints = []
        for constraint_dict in operation.constraints:
            constraint = SpatialConstraint(
                type=constraint_dict["type"],
                subject=constraint_dict["subject"],
                reference=constraint_dict["reference"],
                clearance=constraint_dict.get("clearance", 0.0),
                offset=constraint_dict.get("offset")
            )
            spatial_constraints.append(constraint)
        
        return ObjectPlacement(
            object_id=operation.target_entity,
            object_type=operation.parameters["object_type"],
            constraints=spatial_constraints,
            orientation=operation.parameters.get("orientation")
        )
    
    def _convert_position_robot_operation(self, operation: SymbolicOperation) -> RobotConfiguration:
        """Convert a POSITION_ROBOT operation to a RobotConfiguration."""
        # Convert constraints
        spatial_constraints = []
        for constraint_dict in operation.constraints:
            constraint = SpatialConstraint(
                type=constraint_dict["type"],
                subject=constraint_dict["subject"],
                reference=constraint_dict["reference"],
                clearance=constraint_dict.get("clearance", 0.0),
                offset=constraint_dict.get("offset")
            )
            spatial_constraints.append(constraint)
        
        return RobotConfiguration(
            robot_id=operation.target_entity,
            robot_type=operation.parameters["robot_type"],
            base_position=operation.parameters.get("base_position"),
            base_orientation=operation.parameters.get("base_orientation"),
            joint_config=operation.parameters.get("joint_config", "ready"),
            custom_joints=operation.parameters.get("custom_joints"),
            constraints=spatial_constraints
        )