import numpy as np
import heapq

dr = [-1, -1, -1,  0, 0,  1, 1, 1]
dc = [-1,  0,  1, -1, 1, -1, 0, 1]
MOVE_COST = [np.sqrt(2), 1, np.sqrt(2), 1, 1, np.sqrt(2), 1, np.sqrt(2)]

def isValid(row, col, rowsize, colsize, map):
    return 0 <= row < rowsize and 0 <= col < colsize and map[row][col] < 20  # Ensure it's not an obstacle

def heuristic(curpoint, targetpoint):
    return (targetpoint[0] - curpoint[0]) ** 2 + (targetpoint[1] - curpoint[1]) ** 2

def reconstruct_path(parent_map, start, target):
    path = []
    node = target
    while node != start:
        path.append(node)
        node = parent_map[node]
    path.append(start)
    path.reverse()
    return path

def main():
    map = np.loadtxt('map_data.txt')
    rows, cols = map.shape

    # Find start and target positions
    start_row, start_col, target_row, target_col = -1, -1, -1, -1
    for i in range(rows):
        for j in range(cols):
            if map[i][j] == -8:  # Start position
                start_row, start_col = i, j
            elif map[i][j] == -6:  # Target position
                target_row, target_col = i, j

    if start_row == -1 or target_row == -1:
        print("Start or target not found!")
        return

    # Priority queue: (cost + heuristic, row, col)
    astar = []
    heapq.heappush(astar, (0, start_row, start_col))
    
    cost_map = { (start_row, start_col): 0 }
    parent_map = { (start_row, start_col): None } 

    find_path = False
    while astar:
        cost, currow, curcol = heapq.heappop(astar)

        if (currow, curcol) == (target_row, target_col):
            find_path = True
            break

        for idx in range(8):
            nextrow = currow + dr[idx]
            nextcol = curcol + dc[idx]
            if not isValid(nextrow, nextcol, rows, cols, map):
                continue

            nextcost = cost + MOVE_COST[idx]
            if (nextrow, nextcol) not in cost_map or nextcost < cost_map[(nextrow, nextcol)]:
                cost_map[(nextrow, nextcol)] = nextcost
                parent_map[(nextrow, nextcol)] = (currow, curcol)
                priority = nextcost + heuristic((nextrow, nextcol), (target_row, target_col))
                heapq.heappush(astar, (priority, nextrow, nextcol))

    if find_path:
        print("A path can be found")
        path = reconstruct_path(parent_map, (start_row, start_col), (target_row, target_col))
        print("Path:", path)
        for currow, curcol in path:
            if map[currow][curcol] == -8 or map[currow][curcol] == -6:
                continue
            map[currow][curcol] = -9
        np.savetxt("path.txt", map, fmt="%2d")
        print("Path is saved in path.txt")
    else:
        print("No path can be found")

if __name__ == '__main__':
    main()
