#!/bin/bash
# Problem 1: Basic STRIPS planning (single robot, fixed capacity)

DOMAIN="domain.pddl"
PROBLEM_SMALL="problem1.pddl"  # 2 artifacts
PROBLEM_LARGE="problem2.pddl"  # 6 artifacts

echo "=== PROBLEM 1: Basic STRIPS ==="
echo -e "\n--- Small Problem (2 artifacts) ---"

echo "Testing LAMA-first:"
planutils run downward "--alias lama-first $DOMAIN $PROBLEM_SMALL" 

echo -e "\nTesting LAMA-2011:"
planutils run downward "--alias seq-sat-lama-2011 $DOMAIN $PROBLEM_SMALL"

echo -e "\nTesting FF heuristic (fast greedy search):"
planutils run downward "$DOMAIN" "$PROBLEM_SMALL" "--search eager_greedy([ff()])"

echo -e "\nTesting Additive heuristic (sum of costs):"
planutils run downward  "$DOMAIN" "$PROBLEM_SMALL" "--search eager_greedy([add()])"

echo -e "\nTesting LM-Cut (optimal planning):"
planutils run downward "--alias seq-opt-lmcut $DOMAIN $PROBLEM_SMALL"


echo -e "\n--- Large Problem (6 artifacts) ---"

echo "Testing LAMA-first (scalability test):"
planutils run downward "--alias lama-first $DOMAIN $PROBLEM_LARGE"

echo -e "\nTesting FF heuristic (scalability test):"
planutils run downward  "$DOMAIN" "$PROBLEM_LARGE" "--search eager_greedy([ff()])"

echo -e "\nTesting CG heuristic:"
planutils run downward "$DOMAIN" "$PROBLEM_LARGE" "--search eager_greedy([cg()])"