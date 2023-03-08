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


        
