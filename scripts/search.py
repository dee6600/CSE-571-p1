#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Rushang Karia", "Rashmeet Kaur Nayyar", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Karthik Nelapati"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

from hashlib import new
import math
import os
#import queue
import time
import rospy
import argparse
import traceback
import subprocess
from std_msgs.msg import String

import problem 
from node import Node
from parser import parse_args
from utils import cleanup_ros
from server import generate_maze
from utils import initialize_ros
from priority_queue import PriorityQueue
from server import initialize_search_server


SUBMIT_FILENAME = "hw1_results.csv"
SUBMIT_SEARCH_TIME_LIMIT = 300

class SearchTimeOutError(Exception):
    pass

def is_invalid(state):
    """
        Parameters
        ===========
            state: State
                The state to be checked.
                
        Returns
        ========
            bool
                True if the state is invalid, False otherwise.
    """
    
    return state.x == -1 or state.y == -1


def compute_g(algorithm, node, goal_state):
    """
        Evaluates the g() value.

        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the g-value will be computed.
            node: Node
                The node whose g-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The g-value for the node.
    """

    if algorithm == "bfs":
        
        return node.get_depth()

    if algorithm == "astar":
        
        if node.get_parent():
            return node.get_total_action_cost()
        else:
            return 0
        

    elif algorithm == "gbfs":
        
        return 0


    elif algorithm == "ucs":
        
        if node.get_parent():
            return node.get_total_action_cost()
        else:
            return 0    

    elif algorithm == "custom-astar":
        
        if node.get_parent():
            return node.get_total_action_cost()
        else:
            return 0

    # Should never reach here.
    assert False
    return float("inf")

def manhattan_distance(x1, y1, x2, y2):
    """
        Computes the Manhattan distance between two points.

        Parameters
        ===========
            x1: int
                The x-coordinate of the first point.
            y1: int
                The y-coordinate of the first point.
            x2: int
                The x-coordinate of the second point.
            y2: int
                The y-coordinate of the second point.

        Returns
        ========
            int
                The Manhattan distance between the two points.
    """

    return abs(x1 - x2) + abs(y1 - y2)




def euclidian_distance(x1, y1, x2, y2):
    """
        Computes the Euclidian distance between two points.

        Parameters
        ===========
            x1: int
                The x-coordinate of the first point.
            y1: int
                The y-coordinate of the first point.
            x2: int
                The x-coordinate of the second point.
            y2: int
                The y-coordinate of the second point.

        Returns
        ========
            int
                The Euclidian distance between the two points.
    """

    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def f_bfs(node, goal_state):
    """
        Evaluates the f() value for Best First Search.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """
    

    return compute_g("bfs", node, goal_state)


def f_ucs(node, goal_state):
    """
        Evaluates the f() value for UCS.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    '''
    YOUR CODE HERE
    '''

    return compute_g("ucs", node, goal_state)


def f_astar(node, goal_state):
    """
        Evaluates the f() value for A*.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    x1, y1 = node.get_state().x, node.get_state().y
    x2, y2 = goal_state.x, goal_state.y
    h = manhattan_distance(x1, y1, x2, y2)
    g = compute_g("astar", node, goal_state)

    f = g + h

    return f



def f_gbfs(node, goal_state):
    """
        Evaluates the f() value for GBFS.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    x1, y1 = node.get_state().x, node.get_state().y
    x2, y2 = goal_state.x, goal_state.y

    h = manhattan_distance(x1, y1, x2, y2)

    return h



def f_custom_astar(node, goal_state):
    """
        Evaluates the f() value for A* using a custom heuristic.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    
    

    #####custom heuristic#####
    ##### Experiments #####
    #f = g + h
    #f = g * h
    # f = euclidian distnace from node to init + euclidian distnace from goal to node
    #f = euclidian_distance(x1, y1, 0, 0) + euclidian_distance(x1, y1, x2, y2)
    #f = round(math.log2(g + h))
    #f = node.get_action_cost() + h

    # Thought process
    # After analysing the gbfs and all the other algorithms, I found that the gbfs is the best one.
    # The reaason behind this is that the gbfs is fully depended on the heuristic.
    # So the idea of my experimentation is to increase the effect of the heuristic over g value.
    # we cannot reduce the g value because the g value is the cost of the action, so i tried to increase the h value.


    ### Working Experiemt 1 ###
    ## double the heuristic value
    #f = g + h*2

    ### Working Experiemt 2 ###
    ## square the heuristic value
    #f = g + h**2

    # As I noticed the trend of the above experiemts, i decided to change the heuristic value based on depth of node. 
    # which resulted in follwoing experiemt which is the best one. This heuristic is admissible and is working better than 
    # regulr A* algorithm for every dimesion and every problem.

    ### Working Experiemt 3 ###

    x1, y1 = node.get_state().x, node.get_state().y
    x2, y2 = goal_state.x, goal_state.y

    # Compute the heuristic value with euclidian distance.
    h = euclidian_distance(x1, y1, x2, y2) ** node.get_depth()
    g = compute_g("custom-astar", node, goal_state) 

    f = g + h

    
    


    return f



def graph_search(algorithm, time_limit):
    """
        Performs a graph search using the specified algorithm.

        Maintains two importnt variables:
        1. action_list: It is a list and it contain the sequence of actions to execute to reach from init_state to goal_state.
        2. total_nodes_expanded: It is an integer and it maintains the total number of nodes expanded during the search.
        
        Parameters
        ===========
            algorithm: str
                The algorithm to be used for searching.
            time_limit: int
                The time limit in seconds to run this method for.
                
        Returns
        ========
            tuple(list, int)
                A tuple of the action_list and the total number of nodes
                expanded.
    """
    
    # The helper allows us to access the problem functions.
    helper = problem.Helper()

    # Dictionary to call correct functions to calculate f value of a node
    f_value = {'bfs': f_bfs, 'gbfs': f_gbfs, 'astar': f_astar, 'ucs': f_ucs, 'custom-astar': f_custom_astar}
    
    # Get the initial and the goal states.
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()[0]
    
    # Initialize the init node of the search tree and compute its f_score.
    init_node = Node(init_state, None, 0, None, 0)
    f_score = f_value[algorithm](init_node, goal_state)
    
    # Initialize the fringe as a priority queue.
    priority_queue = PriorityQueue()
    priority_queue.push(f_score, init_node)
    
    # action_list should contain the sequence of actions to execute to reach from init_state to goal_state
    action_list = []

    # total_nodes_expanded maintains the total number of nodes expanded during the search
    total_nodes_expanded = 0
    time_limit = time.time() + time_limit
    
    '''
    YOUR CODE HERE

    Your code for graph search should populate action_list and set total_nodes_expanded
    The automated script will verify their values

    In addition to this you must also write code for:
    1. f_ucs
    2. f_gbfs
    3. f_astar
    4. compute_g
    5. f_custom_astar (If attempting bonus question)
    '''


    # general graph search algorithm

    #empty visited list
    visited_list = []

    #main loop of the algorithm, this will run until the time limit is reached or the goal state is reached. 
    # condition for exit is , if priority queue is empty. 
    while not priority_queue.is_empty():

        #pop the node with the lowest f_score from the priority queue and store it in node
        node = priority_queue.pop()
        
        # check if the node is in visited list or not and also check if the node is valid or not.
        # to conditions are ORed.
        # node will be invalid if the sate of node's x and y is negative. We can impelent greater than also.
        # any one condition is satisfied then the loop will continue.
        if (node.get_state() in visited_list) or is_invalid(node.get_state()):
            continue


        # if the node is valid then add it to the visited list.
        visited_list.append(node.get_state())
        

        # check if the node is the goal state or not.
        if helper.is_goal_state(node.get_state()):

            # running the while loop till the node is not the init/start node.
            # this will be used to get the action list. Reverse order of the action list will be returned.
            # bottom up traversal.
            while node.get_parent() is not None:

                # add the action to the action list.
                action_list.append(node.get_action())

                # set the node to its parent/up node
                node = node.get_parent()

            # reverse the action list to get the correct order from init node to goal node.
            action_list.reverse()

            # return the action list and the total number of nodes expanded.
            ####final output###
            return action_list, total_nodes_expanded

        # we have not found goal state yet.
        # so we will expand the node and add its children to the priority queue.  
        # get the childre/ possible actions of the unexpanded node.  
        possible_actions = helper.get_successor(node.get_state())

        # for each child/action of the node, create a new node and compute its f_score.
        for action in possible_actions:

            # get state and its action cost from possible actions.
            new_state, action_cost = possible_actions[action]

            # create a new node with the new state and parent node.
            #passing new state, parent node,depth from init, action and action cost.
            new_node = Node(new_state, node, node.get_depth() + 1, action, action_cost)

            # compute the f_score of the new node.####
            # f_score will be based on the algorithm used.
            # we will use f_value for priorotising the nodes in the priority queue.
            new_f_score = f_value[algorithm](new_node, goal_state)

            # push the new node to the priority queue with its f_score.
            priority_queue.push(new_f_score, new_node)

        # increment the total number of nodes expanded. Every loop iteration exapnds a node.
        # and we check for all the conditions and apply algorithms accordingly.  
        total_nodes_expanded += 1


        # check if the time limit is reached or not.
        if time.time() >= time_limit:
            raise SearchTimeOutError("Search timed out after %u secs." % (time_limit))

  


def submit(file_handle, env):
    """
        Runs the tests that need to be submitted as a part of this Homework.
        
        Parameters
        ===========
            file_handle: int
                The file descriptor where the results will be output.
    """
    
    SEEDS = [0xDEADC0DE, 0xBADC0D3, 0x500D]

    dim_pairs = {
        'canWorld':[

            # (Grid dimension, Num obstacles)
            # Each grid dimension contains runs with 0%, 10%, 20%, 30%, 40% of max
            # obstacles possible.
            ("4x4", 0), ("4x4", 4),
            ("8x8", 0), ("8x8", 14),
            ("12x12", 0), ("12x12", 31),
            ("16x16", 0), ("16x16", 54)
        ],
        'cafeWorld':[

            # (Grid dimension, Num obstacles)
            ("3x6",0),("3x6",1),("3x6",2),("3x6",4)
        ],

    }

    for env in ['canWorld','cafeWorld']:
        DIMENSION_PAIRS = dim_pairs[env]
        total = len(SEEDS) * len(DIMENSION_PAIRS)
        current = 0
        print("env=%s"% (env) )
        for dimension, obstacles in DIMENSION_PAIRS:
            for seed in SEEDS:
    
                current += 1 
                print("(%3u/%3u) Running dimension=%s, obstacles=%s, seed=%s" % (
                    current,
                    total,
                    dimension,
                    obstacles,
                    seed))
                
                run_search(file_handle, dimension, obstacles, seed, env, algorithms,
                    time_limit=SUBMIT_SEARCH_TIME_LIMIT, debug=False)
   
def run_search(file_handle, dimension, obstacles, seed, env, algorithms, 
    time_limit=float("inf"), debug=True):
    """
        Runs the search for the specified algorithms.
        
        Parameters
        ===========
            file_handle: int
                A descriptor for the output file where the results will be
                written.
            dimension: int
                The dimensions of the grid.
            obstacles: int
                The number of obstacles in the grid.
            seed: int
                The random seed to use in generating the grid.
            algorithms: list(str)
                The algorithms to run.
            time_limit: int
                The time limit in seconds.
            debug: bool
                True to enable debug output, False otherwise.
    """
    
    # Generate the world.
    dimension_x, dimension_y = dimension.split("x")
    dimension_x, dimension_y = int(dimension_x), int(dimension_y)
    generate_maze(dimension_x,dimension_y, obstacles, seed, env)
    
    # Run search for each algorithm.
    for algorithm in algorithms:
    
        error = "None"
        actions = []
        total_nodes_expanded = 0
        start_time = time.time()
        
        # Catch any errors and set the error field accordingly.
        try:
            actions, total_nodes_expanded = graph_search(algorithm, time_limit)
        except NotImplementedError:
        
            error = "NotImplementedError"
        except MemoryError:
        
            error = "MemoryError"
        except Exception as e:
        
            error = str(type(e))
            traceback.print_exc()
            
        time_taken = time.time() - start_time
        time_taken = "%.2f" % (time_taken)
        
        if debug:
        
            print("==========================")
            print("Dimension..........: " + str(dimension))
            print("Obstacles..........: " + str(obstacles))
            print("Seed...............: " + str(seed))
            print("Environment........: " + env)
            print("Algorithm..........: " + algorithm)
            print("Error..............: " + error)
            print("Time Taken.........: " + str(time_taken))
            print("Nodes expanded.....: " + (str(total_nodes_expanded)))
            print("Plan Length........: " + str(len(actions)))
            print("Plan...............: " + str(actions))
        
        if file_handle is not None:
        
            plan_str = '_'.join(action for action in actions)
            file_handle.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (
                dimension, obstacles, seed, algorithm,
                time_taken, total_nodes_expanded, len(actions), error, 
                plan_str,env))


if __name__ == "__main__":

    # Parse the arguments.
    args = parse_args()

    # Check which algorithms we are running.
    if args.algorithm is None or "all" == args.algorithm or args.submit:
    
        algorithms = ["bfs", "ucs", "gbfs", "astar", "custom-astar"]
    else:
    
        algorithms = [args.algorithm]
    
    # Setup the output file.
    if args.output_file is not None:
        
        file_handle = open(args.output_file, "w")
    elif args.submit:
    
        file_name = os.path.join(os.path.dirname(__file__), SUBMIT_FILENAME)
        file_handle = open(file_name, "w")
    else:
    
        file_handle = None
    
    # Write the header if we are writing output to a file as well.
    if file_handle is not None:
    
        file_handle.write("Dimension, Obstacles, Seed, Algorithm, Time, "
            "Nodes Expanded, Plan Length, Error, Plan, Env\n")
    
    # Initialize ROS core.
    roscore_process = initialize_ros()

    # Initialize this node as a ROS node.
    rospy.init_node("search")
    
    # Initialize the search server.
    server_process = initialize_search_server()

    # If using submit mode, run the submission files.
    if args.submit:
    
        submit(file_handle, args.env)
    else:

        # Else, run an individual search.
        run_search(file_handle, args.dimension, args.obstacles, args.seed,
            args.env, algorithms)

    if file_handle is not None:
        file_handle.close()

    # Cleanup ROS core.
    cleanup_ros(roscore_process.pid, server_process.pid)
