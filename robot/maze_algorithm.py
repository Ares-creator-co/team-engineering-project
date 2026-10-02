from collections import deque
import heapq
import time
#first we need to determine what each integer means - this is defined in the documentation 
# 0 = empty cell
# 1 = uncreachable cell e.g wall
# 2 = ending cell
# 3 = visited cell 

#initialising the maze as a list of nested arrays where 0 is an empy cell and 1 is a wall

maze = [
[0, 0, 0, 0, 0, 1], 
[1, 1, 0, 0, 0, 1],    
[0, 0, 0, 1, 0, 0], 
[0, 1, 1, 0, 0, 1], 
[0, 1, 0, 0, 1, 0],
[0, 1, 0, 0, 0, 2]]

def search(x,y): 
    if maze[x][y] == 2:
        print('End found at %d %d ' % (x, y))
        return True
    elif maze[x][y] == 1:
        print('wall at %d %d ' % (x, y))
        return False
    elif maze[x][y] == 3:
        print('This cell %d %d has been visited' % (x, y))
        return False

    print('visiting %d %d' % (x, y))
    #marking it as visited
    maze[x][y] = 3
    # sorting through the grids next to eachother starting clockwise
    if ((x < len(maze) -1 and search(x+1 , y))
        or (y > 0 and search(x, y-1))
        or (x > 0 and search(x-1, y))
        or (y < len(maze[0]) -1 and search(x, y+1))):
        return True
    return False 

search(0,0)
