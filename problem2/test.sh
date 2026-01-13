#!/bin/bash
# Problem 2: Multi-robot with capacities

DOMAIN="domain.pddl"
PROBLEM_SMALL="problem1.pddl"  # α-cap=1, β-cap=2
PROBLEM_LARGE="problem2.pddl"  # both cap=3

echo "=== PROBLEM 2: Multi-Robot Capacity ==="
echo -e "\n--- Small Capacity (alpha=1, beta=2) ---"

echo "=== Testing LAMA-first ==="
planutils run downward "--alias lama-first $DOMAIN $PROBLEM_SMALL"

echo -e "\n=== Testing FF heuristic ==="
planutils run downward "$DOMAIN" "$PROBLEM_SMALL" "--search eager_greedy([ff()])"

echo -e "\n--- Large Capacity (both=3) ---"

echo "=== Testing LAMA-first ==="
planutils run downward "--alias lama-first $DOMAIN $PROBLEM_LARGE"

echo -e "\n=== Testing FF heuristic ==="
planutils run downward  "$DOMAIN" "$PROBLEM_LARGE" "--search eager_greedy([ff()])"

echo -e "\nTesting CG heuristic:"
planutils run downward "$DOMAIN" "$PROBLEM_LARGE" "--search eager_greedy([cg()])"