#!/usr/bin/python
from merlin2_hospital_patrolling.pddl import room_type, room_at, room_patrolled
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from kant_dto import PddlObjectDto, PddlConditionEffectDto
from merlin2_fsm_action import Merlin2FsmAction, Merlin2BasicStates

from yasmin import Blackboard, CbState
from yasmin_ros.basic_outcomes import SUCCEED

import rclpy

class Merlin2RoomPatrolFsmAction(Merlin2FsmAction):
    def __init__(self) -> None:
        self._room = PddlObjectDto(room_type, "room")
        self._wp = PddlObjectDto(wp_type, "wp")

        super().__init__("room_patrol")

        tts_state = self.create_state(Merlin2BasicStates.TTS)
        self.add_state(
            "PREPARING_TEXT",
            CbState([SUCCEED], self.prepare_text),
            transitions={
                SUCCEED: "SPEAKING"
            }
        )
        
        self.add_state(
            "SPEAKING",
            tts_state
        )
    
    def prepare_text(self, blackboard: Blackboard) -> str:
        blackboard.text = f"Strecher room patrolled"
        return SUCCEED

    def create_parameters(self):
        return [self._room, self._wp]

    def create_conditions(self):
        '''
        cond_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            PddlConditionEffectDto.AT_START,
            is_negative = False
        )'''

        cond_2 = PddlConditionEffectDto( # que el robot este en el wp
            robot_at,
            [self._wp],
            PddlConditionEffectDto.AT_START
        )

        cond_3 = PddlConditionEffectDto( 
            room_at,
            [self._room, self._wp],
            PddlConditionEffectDto.AT_START
        )

        return [cond_2, cond_3]

    def create_efects(self):

        effect_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]



def main():
    rclpy.init()
    node = Merlin2RoomPatrolFsmAction()

    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()