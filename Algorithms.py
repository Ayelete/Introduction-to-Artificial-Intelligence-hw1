import numpy as np
from pip._internal.utils.misc import enum

from FrozenLakeEnv import FrozenLakeEnv
from typing import List, Tuple
import heapdict
from enum import Enum

class CASE(enum):
    goal=1
    hole=-1
    frozen=2
    start=0

class Node:
    def __init__(self, state, prev_node, prev_action, cost=0):
        self.state=state
        self.prev_node = prev_node
        self.prev_action=prev_action
        cost = cost
        


class BFSAgent():
    def __init__(self) -> None:
        self.env=None

    def solution(node)->Tuple[List[int],int,float]:
        if node.env.is_final_state:
            return 
    def search(self, env: FrozenLakeEnv) -> Tuple[List[int], int, float]:
        self.env=env
        expanded_nodes=0
        start_node=Node(env.get_initial_state, None, None, 0)
        if start_node.env.is_final_state:
            return solution(node)
        open_list=[start_node]
        close_list=set()

        while open_list:
            curr_node=open_list.pop()
            expanded_nodes+=1
            close_list.add(curr_node.state)

            for i in {1,2,3,4}:
                state,cost,terminated = self.env.succ(node.state)[i]
                child = Node(state, node, i, cost)

                if(child.state not in close_list and child.state not in open_list):
                    if(child.env.is_final_state):
                        return solution(child)
                    elif(terminated):   
                        break
                    else:
                        open_list.add(child)
        return failure
    


class DFSAgent():
    def __init__(self) -> None:
        self.env = None
        self.is_expanded = 0

    def Recursive_DFS_G(self, open_list, close_list, counter_expanded, counter_cost)-> Tuple[List[int], int, float]:
        node = open_list.pop()
        close_list.add(node.state)
        if node.env.is_final_state:
            return solution(node)

        counter_expanded +=1
        counter_cost =+ node.cost_to_arrive

        for i in {1,2,3,4}:
            state,cost,terminated = self.env.succ(node.state)[i]
            child = Node(state, node, i, cost)
            if terminated == 1:
                if child.env.is_final_state:
                    return solution()
                else:
                    return failure

            if child.state not in close_list and child is not in open_list:
                open_list.append(child)
                result = Recurive-DFS-G(open_list, close_list, counter_expanded)
                return result


    def search(self, env: FrozenLakeEnv) -> Tuple[List[int], int, float]:
        counter_expanded = 0
        counter_cost = 0
        self.env = env
        start_node = Node(env.get_initial_state(), None, 0)
        open_list = [start_node]
        close_list = set()
        return self.Recursive_DFS_G(open_list, close_list, counter_expanded, counter_cost)




class UCSAgent():
  
    def __init__(self) -> None:
        raise NotImplementedError

    def search(self, env: FrozenLakeEnv) -> Tuple[List[int], int, float]:
        raise NotImplementedError



class GreedyAgent():
  
    def __init__(self) -> None:
        raise NotImplementedError

    def search(self, env: FrozenLakeEnv) -> Tuple[List[int], int, float]:
        raise NotImplementedError

class WeightedAStarAgent():
    
    def __init__(self):
        raise NotImplementedError

    def search(self, env: FrozenLakeEnv, h_weight) -> Tuple[List[int], int, float]:
        raise NotImplementedError   


class IDAStarAgent():
    def __init__(self) -> None:
        raise NotImplementedError
        
    def search(self, env: FrozenLakeEnv) -> Tuple[List[int], int, float]:
        raise NotImplementedError