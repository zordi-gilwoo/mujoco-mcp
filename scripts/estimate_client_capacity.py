#!/usr/bin/env python3
"""
MuJoCo MCP Client Capacity Estimation Script
Conservative estimation of maximum concurrent clients the server can handle
"""

import psutil
import json
import argparse
import sys
from typing import Dict, Any


def get_system_info() -> Dict[str, Any]:
    """Get detailed system information"""
    try:
        memory = psutil.virtual_memory()
        cpu_info = {
            "physical_cores": psutil.cpu_count(logical=False),
            "logical_cores": psutil.cpu_count(logical=True),
            "cpu_freq": psutil.cpu_freq()._asdict() if psutil.cpu_freq() else None,
        }

        return {
            "memory": {
                "total_gb": round(memory.total / (1024**3), 2),
                "available_gb": round(memory.available / (1024**3), 2),
                "percent_used": memory.percent,
            },
            "cpu": cpu_info,
            "platform": {"system": psutil.os.name, "boot_time": psutil.boot_time()},
        }
    except Exception as e:
        print(f"Error getting system info: {e}")
        return {}


def estimate_resource_usage_per_client() -> Dict[str, Any]:
    """Estimate resource usage per MuJoCo client"""

    # Based on empirical testing and MuJoCo documentation
    estimates = {
        "memory_mb": {
            "mujoco_model": 50,  # MuJoCo model and simulation data
            "physics_engine": 30,  # Physics computation buffers
            "viewer_client": 40,  # Viewer connection and rendering
            "session_overhead": 20,  # Session management, networking
            "python_overhead": 30,  # Python interpreter overhead
            "total": 170,  # Conservative total estimate
        },
        "cpu_percent": {
            "physics_simulation": 3.0,  # Physics stepping and computation
            "viewer_rendering": 1.5,  # Rendering updates (when active)
            "network_io": 0.3,  # Network communication
            "session_management": 0.2,  # Session tracking and cleanup
            "total": 5.0,  # Conservative total estimate
        },
        "network": {
            "ports_per_client": 1,  # Each client gets unique viewer port
            "bandwidth_kbps": 100,  # Estimated network usage per client
        },
        "disk_io": {
            "model_loading_mb": 10,  # One-time model loading
            "logging_mb_per_hour": 5,  # Ongoing logging
        },
    }

    return estimates


def calculate_capacity_limits(
    system_info: Dict[str, Any], usage_estimates: Dict[str, Any]
) -> Dict[str, Any]:
    """Calculate capacity limits based on different constraints"""

    memory = system_info.get("memory", {})
    cpu = system_info.get("cpu", {})

    # Memory constraint
    available_memory_gb = memory.get("available_gb", 4)  # Default 4GB if unknown
    # Reserve 25% of available memory for system and other processes
    usable_memory_gb = available_memory_gb * 0.75
    usable_memory_mb = usable_memory_gb * 1024
    memory_per_client_mb = usage_estimates["memory_mb"]["total"]
    max_clients_memory = int(usable_memory_mb / memory_per_client_mb)

    # CPU constraint
    logical_cores = cpu.get("logical_cores", 4)  # Default 4 cores if unknown
    # Reserve 25% of CPU for system and other processes
    usable_cpu_percent = (logical_cores * 100) * 0.75
    cpu_per_client_percent = usage_estimates["cpu_percent"]["total"]
    max_clients_cpu = int(usable_cpu_percent / cpu_per_client_percent)

    # Network port constraint
    # Assuming ports 8889-9888 (1000 ports available)
    max_clients_ports = 1000

    # Practical limits based on MuJoCo viewer server capabilities
    # MuJoCo viewer server typically handles 10-20 concurrent models well
    max_clients_mujoco_limit = 20

    # Hard cap for safety (conservative approach)
    hard_cap = 50

    # Take the most conservative estimate
    estimated_limits = {
        "memory_limited": max_clients_memory,
        "cpu_limited": max_clients_cpu,
        "port_limited": max_clients_ports,
        "mujoco_limited": max_clients_mujoco_limit,
        "hard_cap": hard_cap,
    }

    # The actual limit is the minimum of all constraints
    max_clients = min(estimated_limits.values())

    return {
        "estimated_max_clients": max_clients,
        "limiting_factor": min(estimated_limits, key=estimated_limits.get),
        "individual_limits": estimated_limits,
        "safety_margins": {
            "memory_reserve_percent": 25,
            "cpu_reserve_percent": 25,
            "hard_cap_applied": max_clients == hard_cap,
        },
    }


def generate_recommendations(capacity_limits: Dict[str, Any], system_info: Dict[str, Any]) -> list:
    """Generate recommendations for improving capacity"""

    recommendations = []
    limits = capacity_limits["individual_limits"]
    limiting_factor = capacity_limits["limiting_factor"]
    max_clients = capacity_limits["estimated_max_clients"]

    if limiting_factor == "memory_limited":
        memory_gb = system_info.get("memory", {}).get("total_gb", 0)
        recommendations.append(
            f"💾 Memory is the limiting factor. Consider adding more RAM (current: {memory_gb}GB)"
        )
        recommendations.append("💾 Optimize MuJoCo model sizes to reduce memory usage per client")

    if limiting_factor == "cpu_limited":
        cpu_cores = system_info.get("cpu", {}).get("logical_cores", 0)
        recommendations.append(
            f"🚀 CPU is the limiting factor. Consider more CPU cores (current: {cpu_cores})"
        )
        recommendations.append("🚀 Reduce physics simulation complexity for better CPU utilization")

    if limiting_factor == "mujoco_limited":
        recommendations.append("🎮 MuJoCo viewer server capacity is the limiting factor")
        recommendations.append(
            "🎮 Consider using headless mode for some clients to reduce viewer load"
        )
        recommendations.append("🎮 Implement viewer connection pooling or sharing")

    if max_clients < 10:
        recommendations.append(
            "⚠️  Very low capacity detected. System may not be suitable for multi-client deployment"
        )
        recommendations.append("⚠️  Consider using a more powerful server or cloud instance")

    if max_clients >= 30:
        recommendations.append("✅ Good capacity for multi-client deployment")
        recommendations.append("✅ Consider implementing auto-scaling based on demand")

    return recommendations


def main():
    parser = argparse.ArgumentParser(description="Estimate MuJoCo MCP client capacity")
    parser.add_argument(
        "--output",
        "-o",
        choices=["text", "json"],
        default="text",
        help="Output format (default: text)",
    )
    parser.add_argument("--detailed", action="store_true", help="Show detailed breakdown")

    args = parser.parse_args()

    print("🔍 MuJoCo MCP Client Capacity Analysis")
    print("=" * 50)

    # Gather system information
    print("Analyzing system resources...")
    system_info = get_system_info()

    # Get resource usage estimates
    usage_estimates = estimate_resource_usage_per_client()

    # Calculate capacity limits
    capacity_limits = calculate_capacity_limits(system_info, usage_estimates)

    # Generate recommendations
    recommendations = generate_recommendations(capacity_limits, system_info)

    # Prepare results
    results = {
        "system_info": system_info,
        "usage_estimates": usage_estimates,
        "capacity_analysis": capacity_limits,
        "recommendations": recommendations,
        "timestamp": psutil.boot_time(),
    }

    if args.output == "json":
        print(json.dumps(results, indent=2))
    else:
        # Text output
        print(f"\n📊 CAPACITY ANALYSIS RESULTS")
        print("-" * 30)
        print(f"Estimated Maximum Clients: {capacity_limits['estimated_max_clients']}")
        print(f"Limiting Factor: {capacity_limits['limiting_factor'].replace('_', ' ').title()}")

        print(f"\n🖥️  SYSTEM RESOURCES")
        print("-" * 20)
        if system_info.get("memory"):
            mem = system_info["memory"]
            print(
                f"Memory: {mem['available_gb']}GB available / {mem['total_gb']}GB total ({mem['percent_used']}% used)"
            )
        if system_info.get("cpu"):
            cpu = system_info["cpu"]
            print(f"CPU: {cpu['logical_cores']} logical cores ({cpu['physical_cores']} physical)")

        if args.detailed:
            print(f"\n📈 RESOURCE LIMITS BREAKDOWN")
            print("-" * 30)
            for limit_type, value in capacity_limits["individual_limits"].items():
                print(f"{limit_type.replace('_', ' ').title()}: {value} clients")

        print(f"\n💡 RECOMMENDATIONS")
        print("-" * 20)
        for rec in recommendations:
            print(f"  {rec}")

        print(f"\n⚠️  IMPORTANT NOTES")
        print("-" * 20)
        print("  • These estimates are conservative and based on typical usage")
        print("  • Actual capacity may vary based on model complexity and usage patterns")
        print("  • Monitor system resources during actual usage for fine-tuning")
        print("  • Consider using the web monitor for real-time capacity tracking")
        print(f"  • Start monitoring server: python launch_monitor.py")


if __name__ == "__main__":
    main()
