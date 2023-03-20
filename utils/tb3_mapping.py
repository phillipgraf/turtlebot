from treelib import Node, Tree

def cell_center(tb3, thresh = 0.2):
    x = tb3.cell[0] - 0.5
    y = tb3.cell[1] - 0.5
    if tb3.VIEW == "north" or tb3.VIEW == "south":
        return y - thresh <= tb3.pos.y <= y + thresh
    if tb3.VIEW == "west" or tb3.VIEW == "east":
        return x - thresh <= tb3.pos.x <= x + thresh

def get_cell(tb3):
    """
    Get the current tb3.cell of the bot.
    IMPORTANT to init the tb3.start_cell before

    :param tb3: Bot object
    :return: The current tb3.cell where the bot is.
    """
    #check tb3.cells on x-axis
    if tb3.pos.x < tb3.cell[0] - 1:
        tb3.cell[0] = tb3.cell[0] - 1
        return tb3.cell
    elif tb3.pos.x > tb3.cell[0]:
        tb3.cell[0] = tb3.cell[0] + 1
        return tb3.cell
    # check tb3.cells on y-axis
    elif tb3.pos.y < tb3.cell[1] - 1:
        tb3.cell[1] = tb3.cell[1] - 1
        return tb3.cell
    elif tb3.pos.y > tb3.cell[1]:
        tb3.cell[1] = tb3.cell[1] + 1
        return tb3.cell
    else:
        return tb3.cell

def get_neighbours_cell(tb3, cell):
    """ Get the location of the neighbour cells of the given cell

    :param tb3: Bot object.
    :param cell: Current cell
    """


    north_neighbour = cell[:]
    north_neighbour[1] = north_neighbour[1] + 1

    south_neighbour = cell[:]
    south_neighbour[1] = south_neighbour[1] - 1

    east_neighbour = cell[:]
    east_neighbour[0] = east_neighbour[0] + 1

    west_neighbour = cell[:]
    west_neighbour[0] = west_neighbour[0] - 1

    tb3.neighbour_cells.update({"north": north_neighbour, "south": south_neighbour, "east": east_neighbour, "west": west_neighbour})

def get_unkown_cells(tb3, laser_dist=1):

    north_neighbour = tb3.neighbour_cells["north"]
    south_neighbour = tb3.neighbour_cells["south"]
    east_neighbour = tb3.neighbour_cells["east"]
    west_neighbour = tb3.neighbour_cells["west"]
    neighbour_cells = []

    if tb3.VIEW == "north":
        if tb3.max_dist_front >= laser_dist and north_neighbour not in tb3.cell_storage and north_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(north_neighbour[:])
            neighbour_cells.append(north_neighbour[:])
        if tb3.max_dist_back >= laser_dist and south_neighbour not in tb3.cell_storage and south_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(south_neighbour[:])
            neighbour_cells.append(south_neighbour[:])
        if tb3.max_dist_left >= laser_dist and west_neighbour not in tb3.cell_storage and west_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(west_neighbour[:])
            neighbour_cells.append(west_neighbour[:])
        if tb3.max_dist_right >= laser_dist and east_neighbour not in tb3.cell_storage and east_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(east_neighbour[:])
            neighbour_cells.append(east_neighbour[:])

    if tb3.VIEW == "south":
        if tb3.max_dist_back >= laser_dist and north_neighbour not in tb3.cell_storage and north_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(north_neighbour[:])
            neighbour_cells.append(north_neighbour[:])
        if tb3.max_dist_front >= laser_dist and south_neighbour not in tb3.cell_storage and south_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(south_neighbour[:])
            neighbour_cells.append(south_neighbour[:])
        if tb3.max_dist_right >= laser_dist and west_neighbour not in tb3.cell_storage and west_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(west_neighbour[:])
            neighbour_cells.append(west_neighbour[:])
        if tb3.max_dist_left >= laser_dist and east_neighbour not in tb3.cell_storage and east_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(east_neighbour[:])
            neighbour_cells.append(east_neighbour[:])

    if tb3.VIEW == "east":
        if tb3.max_dist_left >= laser_dist and north_neighbour not in tb3.cell_storage and north_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(north_neighbour[:])
            neighbour_cells.append(north_neighbour[:])
        if tb3.max_dist_right >= laser_dist and south_neighbour not in tb3.cell_storage and south_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(south_neighbour[:])
            neighbour_cells.append(south_neighbour[:])
        if tb3.max_dist_back >= laser_dist and west_neighbour not in tb3.cell_storage and west_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(west_neighbour[:])
            neighbour_cells.append(west_neighbour[:])
        if tb3.max_dist_front >= laser_dist and east_neighbour not in tb3.cell_storage and east_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(east_neighbour[:])
            neighbour_cells.append(east_neighbour[:])

    if tb3.VIEW == "west":
        if tb3.max_dist_right >= laser_dist and north_neighbour not in tb3.cell_storage and north_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(north_neighbour[:])
            neighbour_cells.append(north_neighbour[:])
        if tb3.max_dist_left >= laser_dist and south_neighbour not in tb3.cell_storage and south_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(south_neighbour[:])
            neighbour_cells.append(south_neighbour[:])
        if tb3.max_dist_front >= laser_dist and west_neighbour not in tb3.cell_storage and west_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(west_neighbour[:])
            neighbour_cells.append(west_neighbour[:])
        if tb3.max_dist_back >= laser_dist and east_neighbour not in tb3.cell_storage and east_neighbour not in tb3.unkown_cells:
            tb3.unkown_cells.append(east_neighbour[:])
            neighbour_cells.append(east_neighbour[:])

    return neighbour_cells
def check_cell(tb3, cell):
    """
    Check if a given tb3.cell is a new tb3.cell for the bot or not.
    Append it to the cell_storage if the cell is new.
    :param tb3: Bot object.
    :param tb3.cell: tb3.cell to be checked
    :return: 
    """

    if cell not in tb3.cell_storage:
        tb3.new_cell = True
        return tb3.new_cell
    elif cell in tb3.cell_storage:
        tb3.new_cell = False
        return tb3.new_cell

def init_tree(tb3):
    """Init the tree to create a map of the maze"""
    tb3.maze = Tree()
    tb3.node_id = tb3.maze.create_node(tb3.cell[:], str(tb3.cell[:])).identifier

def path_creating(tb3):
    """
    Create path as a tree.
    :param tb3: Bot object.
    :return:
    """
    tb3.parent_node_id = tb3.node_id
    if tb3.new_cell:
        tb3.node_id = str(get_cell(tb3))
        tb3.maze.create_node(tb3.cell[:], tb3.node_id, parent=tb3.parent_node_id)

    elif not tb3.new_cell:
        tb3.node_id = str(get_cell(tb3))

def set_cell_counter(tb3, cell, counter_number):
    if str(cell) not in tb3.cell_counters:
        cell_counter = 0 + counter_number
        tb3.cell_counters.update({str(cell): cell_counter})
    else:
        cell_counter = tb3.cell_counters[str(cell)] - 1
        tb3.cell_counters.update({str(cell): cell_counter})
def get_cell_count(tb3, cell):
    return tb3.cell_counters[str(cell)]
