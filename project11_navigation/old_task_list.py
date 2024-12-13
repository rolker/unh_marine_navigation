class OldTaskList:
  '''Keeps track of Task objects'''

  def __init__(self, parent: Task = None) -> None:
    self.parent = parent
    self.tasks: List[Task] = []

    

  def update(self, task_infos: Iterable[TaskInformation]) -> None:
    '''
    Recursively update existing tasks and add new tasks. Existing
    tasks that are not in task_infos are discarded.
    '''

    # list for new or updated tasks in order from task_info
    new_task_list: List[Task] = []
    for task_msg in task_infos:
      id_parts = self.splitChildID(task_msg.id)
      # top level or direct child?
      if len(id_parts[0]) == 0 and len(id_parts[1]) > 0:
        task = None
        for existing_task in self.tasks:
          if existing_task.task_information.id == task_msg.id:
            task = existing_task
            break
        if task is None:
          task = Task(task_msg, self.parent)
        task.update(task_infos)
        new_task_list.append(task)
    self.tasks = new_task_list
        

  def taskMessages(self) -> List[TaskInformation]:
    '''
    Returns a list of TaskInformation messages, recursing 
    down the children. 
    '''
    task_messages = []
    children_messages = []
    for task in self.tasks:
      task_messages.append(copy.deepcopy(task.task_information))
      children_messages += task.children.taskMessages()
    return task_messages + children_messages

  def tasksByPriority(self, skip_done: bool = False) -> List[Task]:
    '''
    Returns a list of direct children Tasks sorted by priority.
    '''
    priority_map = {}
    for task in self.tasks:
      if not skip_done or not task.done():
        priority = task.task_information.priority
        if not priority in priority_map:
          priority_map[priority] = []
        priority_map[priority].append(task)

    prioritized_tasks = []
    for priority in sorted(priority_map):
      for task in priority_map[priority]:
        prioritized_tasks.append(task)
    
    return prioritized_tasks


  def getFirstPose(self, recursive: bool = False) -> PoseStamped:
    '''
    Returns the first pose in the task list. Optionally
    search recursively (depth first).
    '''
    for task in self.tasks:
      pose = task.getFirstPose(recursive)
      if pose is not None:
        return pose
    return None

  def getLastPose(self, recursive: bool = False) -> PoseStamped:
    '''
    Returns the first pose in the task list. Optionally
    search recursively (depth first).
    '''
    for task in reversed(self.tasks):
      pose = task.getLastPose(recursive)
      if pose is not None:
        return pose
    return None

  def createTaskBefore(self, task: Task = None, type: str = "") -> Task:
    '''
    Creates a new task and optionally inserts it before task.
    If task is not None and it is not found, no new task is created.
    Returns the new task or None.
    '''
    insertion_spot = None
    if task is not None:
      for potential_task in enumerate(self.tasks):
        if potential_task[1].task_information.id == task.task_information.id:
          insertion_spot = potential_task[0]
          break
      if insertion_spot is None:
        return None
    
    task_info = TaskInformation()
    task_info.type = type
    if task is not None:
      task_info.priority = task.task_information.priority
    new_task = Task(task_info, self.parent)
    if insertion_spot is not None:
      self.tasks.insert(insertion_spot, new_task)
    else:
      self.tasks.append(new_task)
    return new_task


  def allDone(self, recursive: bool = False) -> bool:
    '''
    Returns true if all tasks in the list are done,
    optionally recursing down the children.
    '''
    for task in self.tasks:
      if(not task.done(recursive)):
        return False
    return True

  def generateUniqueID(self, prefix: str, skip: Task = None) -> str:
    '''
    Generates a unique id for a child task using
    the parent id (if not None) and prefix with a number
    appended. The first number from 0 to 999 resulting
    in a unique id is used. None is returned if
    a unique id can't be found. If a skip task is supplied
    allow its id to be reused.
    '''
    path = ''
    if self.parent is not None:
      path = self.parent.task_information.id + '/'
    for i in range(1000):
      candidate = path+prefix
      if i < 10:
        candidate += '00'
      elif i < 100:
        candidate += '0'
      candidate += str(i)
      exists = False
      for task in self.tasks:
        if task.task_information.id == candidate:
          if skip is None or task.task_information.id != skip.task_information.id:
            exists = True
            break
      if not exists:
        return candidate
    return None

  def splitChildID(self, task_id: str) -> Tuple[str, str]:
    '''
    Split the id of a potential child task into the path and
    direct child part. Returns None if the id is not
    for a child task.
    '''
    parent_path = ""
    if self.parent is not None:
      parent_path = self.parent.task_information.id + '/'
    if self.parent is None or task_id.startswith(parent_path):
      child_part = task_id[len(parent_path):]
      return child_part.rsplit('/', 1)

    return None
