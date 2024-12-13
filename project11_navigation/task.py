#!/usr/bin/env python3

from typing import Iterable, Any, List, Tuple, Dict

import rospy
from project11_nav_msgs.msg import TaskInformation

import io
import copy
import yaml

def parentID(task_id: str) -> str:
  if '/' in task_id:
    return task_id.rsplit('/',1)[0]
  return None


class Task:
  '''Help manage TaskInformation messages'''

  def __init__(self, task_information: TaskInformation, task_list: 'TaskList') -> None:
    '''
    Initializes a Task, adding or updating entries in task_list.
    '''
    self.task_information = copy.deepcopy(task_information)
    self.task_list = task_list
    self.last_update_time = rospy.Time.now()
    task_list.tasks[task_information.id] = self
    if not task_information.id in task_list.task_order_ids:
      task_list.task_order_ids.append(task_information.id)
 

  def parentID(self) -> str:
    '''
    Returns the id of the parent task if a child or
    None if this is a top-level task
    '''
    return parentID(self.task_information.id)


  def parent(self) -> 'Task':
    return self.task_list.parentTask(self.task_information.id)


  def children(self) -> List['Task']:
    return self.task_list.childrenTasks(self.task_information.id)

  def update(self, task_info: TaskInformation) -> None:
    '''
    Updates TaskInformation message for this task.
    '''
    if self.task_information.id != task_info.id:
      raise KeyError("Trying to update a task with a different id")
    self.task_information = copy.deepcopy(task_info)
    self.last_update_time = rospy.Time.now()
    
  def sameMessage(self, task_information: TaskInformation) -> bool:
    '''Checks if the incoming TaskInformation message is identical
    to this task's version.
    '''
    this_buffer = io.StringIO()
    self.task_information.serialize(this_buffer)
    other_buffer = io.StringIO()
    task_information.serialize(other_buffer)
    return this_buffer.getvalue() == other_buffer.getvalue()
  
  def message(self) -> TaskInformation:
    '''Returns the TaskInformation message'''
    return self.task_information
  
  def descendantMessages(self) -> List[TaskInformation]:
    '''
    Returns a list of TaskInformation messages for all
    the children tasks recursively.
    '''
    messages = []
    for task in self.children():
      messages.append(task.message())
      messages += messages
    return messages

  def done(self, recursive: bool = False) -> bool:
    '''Returns true if task and optionally the children tasks
    are done. Defaults to not checking children.
    '''
    if not recursive:
      return self.task_information.done
    
    return self.task_information.done and self.task_list.allChildrenDone(self.task_information.id)

  def setDone(self) -> None:
    '''Set the message's done flag to True'''
    self.task_information.done = True
    self.last_update_time = rospy.Time.now()

  def getFullChildID(self, child_local_id: str) -> None:
    '''Generates a full id for a child task of this task.
    The full id is built by adding the id parameter to this task's id and a separating '/'.
    '''
    return self.task_information.id+'/'+child_local_id

  def data(self) -> Any:
    '''Return's the TaskInformation's data decoded from YAML'''
    return yaml.safe_load(self.task_information.data)

  def setData(self, data: Any) -> None:
    '''YAML encodes data and sets it'''
    self.task_information.data = yaml.safe_dump(data)
    self.last_update_time = rospy.Time.now()

  def dataItem(self, key: str, recurse_up: bool = True) -> Any:
    '''
    Returns a data item from the parsed data member of the message
    optionally recursing up the parents.
    '''
    data = self.data()
    if key in data:
      return data[key]
    if recurse_up:
      return self.task_list.parentTask(self.task_information.id).dataItem(key, recurse_up)
    return None

  def status(self) -> Any:
    '''Return's the TaskInformation's status data decoded from YAML'''
    return yaml.safe_load(self.task_information.status)

  def setStatus(self, status: Any) -> None:
    '''YAML encodes status and sets it'''
    self.task_information.status = yaml.safe_dump(status)
    self.last_update_time = rospy.Time.now()

# this is to make type hints work
from project11_navigation.task_list import TaskList
