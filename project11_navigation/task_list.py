#!/usr/bin/env python3

from typing import Dict, List

from project11_navigation.task import Task, parentID

from project11_nav_msgs.msg import TaskInformation

import copy

def isTopLevel(task_id: str) -> bool:
  '''
  Returns True if task_id refers to a top level task
  '''
  return not '/' in task_id


class TaskList:
  '''
  Keeps track of tasks and helps with relationships
  '''

  def __init__(self) -> None:
    self.tasks: Dict[str, Task] = {}
    self.task_order_ids: List[str] = []

  def clear(self) -> None:
    self.tasks.clear()
    self.task_order_ids.clear()

  def update(self, task_info: TaskInformation) -> None:
    '''
    Updates task's TaskInformation. 
    '''
    if not task_info.id in self.tasks:
      raise KeyError("Task not found with id "+task_info.id)
    self.tasks[task_info.id] = copy.deepcopy(task_info)

  def task(self, task_id: str) -> Task:
    '''
    Return the task corresponding to the task_id.
    '''
    if task_id is self.tasks:
      return self.tasks[task_id]
    return None

  def parentTask(self, task_id: str) -> 'Task':
    '''
    Find and return a parent task if it exists, or None
    '''
    
    if not isTopLevel(task_id):
      return self.tasks[parentID(task_id)]
    return None
  
  def addOrUpdate(self, task_information: TaskInformation, prepend: bool = False) -> 'Task':
    '''
    Adds new task or updates an existing task.
    '''

    if task_information.id in self.tasks:
      task = self.tasks[task_information.id]
      task.update(task_information)
      return task
    if prepend:
      self.task_order_ids.insert(0, task_information.id)
    else:
      self.task_order_ids.append(task_information.id)
    return Task(task_information, self)

  def listMessages(self) -> List[TaskInformation]:
    '''
    Returns a list of TaskInformation messages
    '''
    messages: List[TaskInformation] = []
    for id in self.task_order_ids:
      messages.append(self.tasks[id].task_information)
    return messages

  def addOrUpdateMany(self, tasks: List[TaskInformation], prepend: bool = False) -> None:
    '''
    Add or updates tasks. Returns list of ids of affected tasks.
    '''
    task_id_list: List[str] = []
    for task in tasks:
      self.addOrUpdate(task, prepend)
      task_id_list.append(task.id)
    return task_id_list
  
  def clearExcept(self, task_ids: List[str]) -> int:
    '''
    Remove all the tasks except those listed in task_ids.
    Returns number of tasks removed.
    '''
    new_task_order_ids = []
    discard_list = []
    for task_id in self.tasks:
      if task_id in task_ids:
        new_task_order_ids.append(task_id)
      else:
        discard_list.append(task_id)
    for discard_id in discard_list:
      self.tasks.pop(discard_id, None)
    self.task_order_ids = new_task_order_ids
    return len(discard_list)

  def remove(self, task_id: str) -> 'Task':
    '''Removes and returns task if it exists or None'''
    if task_id in self.task_order_ids:
      self.task_order_ids.remove(task_id)
    return self.tasks.pop(task_id, None)

  def childrenOf(self, parent_id: str) -> List[Task]:
    '''
    Returns list of children of parent tasks identified by parent_id.
    '''
    children = []
    parent_path = parent_id+'/'
    for task_id in self.task_order_ids:
      if task_id.startswith(parent_path):
        children.append(self.tasks[task_id])
    return children

  def allChildrenDone(self, parent_id: str) -> bool:
    for child_task in self.childrenOf(parent_id):
      if not child_task.done():
        return False
    return True