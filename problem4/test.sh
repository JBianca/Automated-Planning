#!/bin/bash
# Problem 4: Temporal Planning

DOMAIN="domain.pddl"
PROBLEM="problem.pddl"

echo "=== PROBLEM 4: Temporal Planning ==="

echo -e "\n=== Testing POPF  (Partial Order Planning Forward):"
planutils run popf "$DOMAIN" "$PROBLEM"

echo -e "\n=== Testing TFD (Temporal Fast Downward):"
planutils run tfd "$DOMAIN" "$PROBLEM"

echo -e "\n=== Testing OPTIC with makespan optimization:"
planutils run optic "-N $DOMAIN $PROBLEM"

echo -e "\n=== Testing OPTIC with different search strategies:"
echo " \n=== Testing   a) SAT-based:"
planutils run optic "-N -S $DOMAIN $PROBLEM"

echo -e "\n=== Testing   b) Graphplan-based:"
planutils run optic "-N -G $DOMAIN $PROBLEM"