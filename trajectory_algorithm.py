import math
from vpython import vector
from shapely.geometry import Point, Polygon, LineString
import numpy as np

def trajectory_algorithm(w_t, o_t, b_t):
    # coordinates of plane
    obstacle_setting = 50
    distance_setting = 30

    p_t = w_t[0]

    # coordinates of start and goal points during path search for each waypoint
    global s_t
    global g_t

    # list of nodes' information
    nodes = []

    # lists of selected paths
    paths_list = []
    export_path = []

    # currently focused waypoint
    focused_waypoint = 1

    # distance between two 2D points
    def dis(t1, t2):
        return math.hypot(t2[0] - t1[0], t2[1] - t1[1])

    # determinant of two 2D vectors
    def det(t1, t2):
        return t1[0] * t2[1] - t1[1] * t2[0]

    '''
    # intersection of two 2D lines
    def line_intersection(t1, t2, t3, t4):
        # compute the lines' cartesian coefficients
        k = 10
        line = inf_line(t1, t2, k).intersection(inf_line(t3, t4, k))
        while line.is_empty:
            line = inf_line(t1, t2, k).intersection(inf_line(t3, r4, k))
            k *= 100
        # return the x or y
        return line.x, l.y

    # make line infinite
    def inf_line(t1, t2, k):
        a1, b1 = t1
        a2, b2 = t2
        if a1 == a2:
            b2 = k
            b1 = -k
        else:
            m = (b2 - b1) / (a2 - a1)
            a1 += k
            a2 += -k
            b1 += k * m
            b2 += -k * m
        t1 = (a1, b1)
        t2 = (a2, b2)
        return LineString([t1, t2])
    '''

    # check if point is inside segment
    def point_in_segment(ti, t1, t2):
        return dis(ti, t1) + dis(ti, t2) < dis(t1, t2) + 1

    def segment_cross(t1, t2, t3, t4):
        return not LineString([t1, t2]).intersection(LineString([t3, t4])).is_empty

    # check if point is inside an obstacle
    def is_not_in_obstacle(t):
        valid = True
        # scan through obstacles and check if distance between their center and the point is smaller than their radius
        for io in o_t:
            if valid:
                oot, oor = io
                if dis(oot, t) < oor + 1:
                    valid = False
        return valid

    # project a point on a line
    def point_line_projection(ti, t1, t2):
        vi = vector(ti[0], ti[1], 0)
        v_1 = vector(t1[0], t1[1], 0)
        v_2 = vector(t2[0], t2[1], 0)
        line_vec = v_2 - v_1
        outer_vec = vi - v_1
        final_vec = v_1 + outer_vec.proj(line_vec)

        return final_vec.x, final_vec.y

    # check if a path from two points does not cross the border or an obstacle
    def is_valid_path(t1, t2):
        valid = True
        # compute the line between the points' cartesian coefficients
        if t1 != t2:
            for io in o_t:
                if valid:
                    ooot, ooor = io
                    ct = point_line_projection(ooot, t1, t2)
                    # check if the center of the obstacle and the line are closer than the radius of the obstacle
                    if dis(ct, ooot) < (ooor + distance_setting) and point_in_segment(ct, t1, t2):
                        valid = False
                    # check if either of the points is inside the obstacle
                    if dis(ooot, t1) < ooor or dis(ooot, t2) < ooor:
                        valid = False

            # scan through the border's points
            for border_index in range(len(b_t)):
                if valid:
                    # get a point and the one before it
                    tb1 = b_t[border_index]
                    tb2 = b_t[border_index - 1]

                    # check if the two points' segment cross with the border's segment
                    if segment_cross(t1, t2, tb1, tb2):
                        valid = False

        return valid

    # push away a point from a corner of the border depending on a factor
    def push_away(border_index, factor):
        b_i = b_t[border_index]
        v = t_to_v(b_i)

        # find the neighbors of the border point
        if border_index == len(b_t) - 1:
            v_1 = t_to_v(b_t[0])
        else:
            v_1 = t_to_v(b_t[border_index + 1])

        v_2 = t_to_v(b_t[border_index - 1])

        # compute the vectors from the border point to its neighbors, turn them into unit vectors, average them and make
        # a point with them

        iv = v + (factor / 2) * ((v_1 - v).norm() + (v_2 - v).norm())

        return iv.x, iv.y

    '''
    # push away a point from a corner of the border depending on a factor
    def push_away(border_index, factor):
        vx, vy = b_t[border_index]

        # find the neighbors of the border point
        if border_index == len(b_t) - 1:
            vx1, vy1 = b_t[0]
        else:
            vx1, vy1 = b_t[border_index + 1]

        vx2, vy2 = b_t[border_index - 1]

        # compute the vectors from the border point to its neighbors, turn them into unit vectors, average them and make
        # a point with them
        d1 = dis((vx1, vy1), (vx, vy))
        d2 = dis((vx2, vy2), (vx, vy))

        vx1 = (factor * (vx1 - vx)) / d1 + vx
        vx2 = (factor * (vx2 - vx)) / d2 + vx
        vy1 = (factor * (vy1 - vy)) / d1 + vy
        vy2 = (factor * (vy2 - vy)) / d2 + vy
        return (vx1 + vx2) / 2, (vy1 + vy2) / 2
    '''

    def t_to_v(t):
        return vector(t[0], t[1], 0)

    # change the information of a node
    def m_write(n_index, info_type, value):
        list_version = list(nodes[n_index])
        list_version[info_type] = value
        nodes[n_index] = tuple(list_version)

    # compute a node k's score if it came from a parent node e
    def new_score(node_e, node_k_t):
        # distance crossed from start to the node k
        return node_e[1] + dis(node_e[2], node_k_t) + dis(g_t, node_k_t) - dis(g_t, node_e[2])

    def is_good_path(node_e, n_t):
        return is_useful_path(node_e, n_t) and is_valid_path(node_e[2], n_t)


    def is_useful_path(node_e, n_t):
        passed = True
        node_t = node_e[2]
        p_node_i = node_e[3]

        # if the current node e has a parent node:
        if p_node_i > float("-inf"):
            node_p_t = nodes[p_node_i][2]
            special_index = node_e[4]
            old = t_to_v(node_p_t)
            current = t_to_v(node_t)
            new = t_to_v(n_t)
            # compute vectors from parent node to current node, then to target node
            old_to_current = old - current
            current_to_new = current - new
            m2 = np.array([[current_to_new.x, current_to_new.y], [old_to_current.x, old_to_current.y]])

            # if the node is around an obstacle, get its coordinates and compute a vector from the parent node to it
            if special_index > 0 and passed:
                if old_to_current.dot(current_to_new) < 0:
                    passed = False
                obstacle = t_to_v(o_t[special_index - 1][0])

                old_to_obstacle = old - obstacle
                m1 = np.array([[old_to_obstacle.x,old_to_obstacle.y],[old_to_current.x,old_to_current.y]])
                # check if the path is not straight and bending around the obstacle with a determinant
                if passed:
                    if np.linalg.det(m1) * np.linalg.det(m2) < 0:
                        passed = False

            # if the node is around a concave border point, get its coordinates/compute a vector from the parent node
            # to it
            if special_index < 0 and passed:

                border = t_to_v(b_t[-special_index - 1])

                old_to_border = old - border
                m1 = np.array([[old_to_border.x, old_to_border.y], [old_to_current.x, old_to_current.y]])


                # check if the path is not straight and bending around the border point with a determinant
                if np.linalg.det(m1) * np.linalg.det(m2) < 0:
                    passed = False

            # scan through the path from the start to the current node and check if it crosses itself with the new node

            iiit2 = node_p_t

            while p_node_i > 0 and passed:
                iiit1 = iiit2
                iiit2 = nodes[nodes[p_node_i][3]][2]
                if segment_cross(iiit1, iiit2, nodes[e][2], n_t):
                    passed = False
                p_node_i = nodes[p_node_i][3]

        return passed
    '''





    # check if path from the current node e to next node is useful
    def is_useful_path(node_e, n_t):
        passed = True
        node_t = node_e[2]
        p_node_i = node_e[3]

        # if the current node e has a parent node:
        if p_node_i > float("-inf"):
            node_p_t = nodes[p_node_i][2]
            special_index = node_e[4]
            old = t_to_v(node_p_t)
            current = t_to_v(node_t)
            new = t_to_v(n_t)

            # compute vectors from parent node to current node, then to target node
            old_to_current_x = node_p_t[0] - node_t[0]
            old_to_current_y = node_p_t[1] - node_t[1]
            current_to_new_x = node_t[0] - n_t[0]
            current_to_new_y = node_t[1] - n_t[1]
            old_to_current_t = (old_to_current_x, old_to_current_y)
            current_to_new_t = (current_to_new_x, current_to_new_y)
            old_to_current = old - current
            current_to_new = current - new
            # if the node is around an obstacle, get its coordinates and compute a vector from the parent node to it
            if special_index > 0 and passed:
                if old_to_current.dot(current_to_new) < 0:
                    passed = False
                iio_x, iio_y = o_t[special_index - 1][0]

                old_to_obstacle_t = (node_p_t[0] - iio_x, node_p_t[1] - iio_y)

                # check if the path is not straight and bending around the obstacle with a determinant
                if passed:
                    if det(old_to_obstacle_t, old_to_current_t) * det(current_to_new_t, old_to_current_t) < 0:
                        passed = False

            # if the node is around a concave border point, get its coordinates/compute a vector from the parent node
            # to it
            if special_index < 0 and passed:

                ib_x, ib_y = b_t[-special_index - 1]

                old_to_border_t = (node_p_t[0] - ib_x, node_p_t[1] - ib_y)

                # check if the path is not straight and bending around the border point with a determinant
                if det(old_to_border_t, old_to_current_t) * det(current_to_new_t, old_to_current_t) < 0:
                    passed = False

            # scan through the path from the start to the current node and check if it crosses itself with the new node

            iiit2 = node_p_t

            while p_node_i > 0 and passed:
                iiit1 = iiit2
                iiit2 = nodes[nodes[p_node_i][3]][2]
                if segment_cross(iiit1, iiit2, nodes[e][2], n_t):
                    passed = False
                p_node_i = nodes[p_node_i][3]

        return passed
    '''
    # if possible, add a node around an obstacle, and make its fixed parent the previous node
    def add_obstacle_node(eeee, oot, o_index):
        if is_good_path(nodes[eeee], oot) and oot != nodes[eeee][2]:
            nodes.append((1, new_score(nodes[eeee], oot), oot, eeee, o_index + 1, 1))

    # update the map with new nodes and start/goal points when looking for a new waypoint's path
    def map_update():
        # clear previous node graph
        nodes.clear()
        global s_t
        global g_t
        # set start and end points
        if focused_waypoint == 0:
            s_t = p_t
        else:
            s_t = w_t[focused_waypoint - 1]

        g_t = w_t[focused_waypoint]

        # node information:
        #   <research state, score, x, y, parent node, obstacle or border node, fixed node>
        #   research state: 0 = unexplored; 1: open; 2: closed
        #   score: total distance crossed from start to the end if passing through this node and its parents
        #   x, y: coordinates
        #   obstacle or border node: 0 if default node; obstacle index + 1 if built on obstacle;
        #                            -border index -1 if built on border point
        #   fixed node: 0 if it can change its parent node, 1 if it can't

        # add the start point to the graph of nodes
        nodes.append((1, dis(s_t, g_t), s_t, float("-inf"), 0, 0))
        # add nodes for concave border points if possible
        if len(b_t) > 2:
            for border_index in range(len(b_t)):
                tp = push_away(border_index, -60)
                if border_poly.contains(Point(tp)) and is_not_in_obstacle(tp):
                    nodes.append((0, float("+inf"), tp, float("+inf"), -(border_index + 1), 0))

    if len(b_t) > 2:
        border_poly = Polygon(b_t)

    # clean previous path list and build first node graph
    map_update()

    redo = True
    # print("----------")
    # print("----------")
    # repeat the search process until the end (found path or took too many nodes)
    while redo:
        # print("----------")
        # for i in range(len(nodes)):
        #    print(nodes[i])
        # if the algorithm has to jump to searching for the next waypoint's path
        next_waypoint = False

        # find the open node with the lowest score
        min_score = float("inf")
        # e is the current focused-on node
        e = 0
        found = 0

        for node_index in range(len(nodes)):
            if nodes[node_index][1] < min_score and nodes[node_index][0] == 1:
                min_score = nodes[node_index][1]
                e = node_index
                found = 1

        # if there are not open nodes, or too many generated nodes, move on to the next waypoint
        if found == 0 or len(nodes) > 1500:
            next_waypoint = True
        node_e_e = nodes[e]
        node_e_t = node_e_e[2]
        # if there is a direct useful path to the waypoint, add the whole path to the list and move on to the next
        # waypoint
        if is_good_path(node_e_e, g_t):
            # go back the way of the node's parents to build the path from the start
            memory_path = [g_t]
            to_add = e
            while to_add > float("-inf"):
                memory_path.insert(0, nodes[to_add][2])

                to_add = nodes[to_add][3]

            paths_list.append(memory_path)

            next_waypoint = True

        # if still in the search process for the waypoint:
        if not next_waypoint:

            # generate nodes around obstacles coming from the current node
            for obstacle_index in range(len(o_t)):
                print(len(nodes))
                ot, io_r = o_t[obstacle_index]
                o_vec = t_to_v(ot)
                n_vec = t_to_v(node_e_t)
                # compute points perpendicular to the vector from the current node to the obstacle on each side
                s = (io_r + obstacle_setting) * (o_vec - n_vec).norm()

                angle_test1 = 0
                angle_test2 = 0
                ns = vector(s.y, -s.x, 0)
                v1 = o_vec + ns
                v2 = o_vec - ns

                # bring the nodes closer by changing the angle around the obstacle until they make a valid path
                while (not is_valid_path((v1.x, v1.y), node_e_t)) and angle_test1 < math.tau / 4:
                    v1 = o_vec - (+ns).rotate(angle_test1, vector(0, 0, 1))
                    angle_test1 += math.tau / 72
                while (not is_valid_path((v2.x, v2.y), node_e_t)) and angle_test2 < math.tau / 4:
                    v2 = o_vec - (-ns).rotate(-angle_test2, vector(0, 0, 1))
                    angle_test2 += math.tau / 72

                add_obstacle_node(e, (v1.x, v1.y), obstacle_index)
                add_obstacle_node(e, (v2.x, v2.y), obstacle_index)

            # change all the dynamic nodes' parent to the current node if it lowers their score
            for node_index in range(len(nodes)):
                node_i = nodes[node_index]
                node_i_t = node_i[2]
                if node_i[0] != 2 and node_i[5] != 1:
                    new_score_2 = new_score(node_e_e, node_i_t)
                    if new_score_2 < node_i[1]:
                        if is_good_path(node_e_e, node_i_t):
                            m_write(node_index, 0, 1)
                            m_write(node_index, 1, new_score_2)
                            m_write(node_index, 3, e)

            # close the current node
            m_write(e, 0, 2)

        # if there is another waypoint, focus the search on it and update the map, if not, stop the search
        else:
            if focused_waypoint + 1 < len(w_t):
                focused_waypoint += 1
                map_update()

            else:
                redo = False
                focused_waypoint = 0

    # type the path list
    if len(paths_list) > 0:
        export_path.append(paths_list[0][0])
        for path in paths_list:
            export_path.extend(path[1:])

    return export_path
