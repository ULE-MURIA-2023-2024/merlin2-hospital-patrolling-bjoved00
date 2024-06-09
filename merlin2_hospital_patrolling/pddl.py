#!/usr/bin/python
from merlin2_basic_actions.merlin2_basic_types import wp_type
from kant_dto import PddlTypeDto, PddlPredicateDto#encapsula en python todos los elementos de PDDL 

room_type = PddlTypeDto("room")
room_patrolled = PddlPredicateDto("room_patrolled", [room_type])
room_at = PddlPredicateDto("room_at", [room_type, wp_type])
