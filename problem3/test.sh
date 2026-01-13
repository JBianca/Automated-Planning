#!/bin/bash
# Problem 3: HTN Planning

HTN_DOMAIN="domain.pddl"
HTN_PROBLEM="problem.pddl"

echo "=== PROBLEM 3: HTN Planning ==="

echo -e "Testing PANDA (HTN planner):"
java -jar ../PANDA.jar -parser hddl domain.hddl problem.hddl
