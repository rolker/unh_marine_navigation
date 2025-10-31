#!/usr/bin/python3

'''
Written by Val Schmidt with contributions from Roland Arsenault
'''

import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import OccupancyGrid
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList, GeoVizPolygon
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geographic_msgs.msg import GeoPoint
import project11

def costmap_callback(data):
    origin = PointStamped()
    origin.point = data.info.origin.position
    origin.header = data.header

    height_meters = data.info.resolution*data.info.height
    width_meters = data.info.resolution*data.info.width

    opposite = PointStamped()
    opposite.point.x = origin.point.x+height_meters
    opposite.point.y = origin.point.y+width_meters
    opposite.point.z = origin.point.z
    opposite.header = data.header

    corners_ll = earth.pointListToGeoPointList((origin, opposite))
    if len(corners_ll) == 2 and corners_ll[0] is not None and corners_ll[1] is not None:
        origin_ll = corners_ll[0].position
        opposite_ll = corners_ll[1].position

        vizItem = GeoVizItem()
        
        vizItem.id = 'occupency_grid'
        plist = GeoVizPointList()
        plist.color.r = 0.0
        plist.color.g = 0.5
        plist.color.b = 1.0
        plist.color.a = 1.0
        corners = [origin_ll,opposite_ll]
        for i in ( (0,0), (0,1), (1,1), (1,0), (0,0) ):
            gp = GeoPoint()
            gp.latitude = corners[i[0]].latitude
            gp.longitude = corners[i[1]].longitude
            plist.points.append(gp)
        vizItem.lines.append(plist)
        
        dlat = (opposite_ll.latitude - origin_ll.latitude)/float(data.info.width)
        dlong = (opposite_ll.longitude - origin_ll.longitude)/float(data.info.height)
        

        for row in range(data.info.height):
            for col in range(data.info.width):
                if data.data[row*data.info.width+col] > 0:
                    intensity = data.data[row*data.info.width+col]/255.0
                    p = GeoVizPolygon()
                    p.fill_color.r = 0.1
                    p.fill_color.g = 0.1
                    p.fill_color.b = 0.1
                    p.fill_color.a = intensity
                    for i in ( (0,0), (0,1), (1,1), (1,0), (0,0) ):
                        gp = GeoPoint()
                        gp.latitude = origin_ll.latitude+dlat*(row+i[0])
                        gp.longitude = origin_ll.longitude+dlong*(col+i[1])
                        p.outer.points.append(gp)

                    vizItem.polygons.append(p)
        display_publisher.publish(vizItem)
        
grid_sub = None
# allow tf_buffer to fill up a bit before asking for a grid
def delayed_subscribe(data):
    global grid_sub
    grid_sub = rospy.Subscriber('occupancy_grid', OccupancyGrid, costmap_callback, queue_size=1)

rospy.init_node("occupancy_to_camp")

earth = project11.nav.EarthTransforms()


tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

display_publisher = rospy.Publisher('project11/display', GeoVizItem, queue_size = 10)

rospy.Timer(rospy.Duration(2), delayed_subscribe, oneshot=True)

rospy.spin()
