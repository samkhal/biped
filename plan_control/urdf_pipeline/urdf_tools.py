import xml.etree.ElementTree as ET
import os

# Eliminate all links and other elements down the chain
def remove_chain(robot, links_to_remove):

	while links_to_remove:
	    remove_link = links_to_remove.pop()
	    
	    for link in robot.findall('link'):
	        if link.attrib['name']==remove_link:
	            robot.remove(link)
	    
	    joints_to_remove = []
	    for joint in robot.findall('joint'):
	        parent = joint.find('parent').attrib['link']
	        child = joint.find('child').attrib['link']
	        if child == remove_link:
	            joints_to_remove.append(joint)
	        elif parent == remove_link:
	            links_to_remove.append(child)
	            joints_to_remove.append(joint)
	    
	    for joint in joints_to_remove:
	        robot.remove(joint)
	        
	    for transmission in robot.findall('transmission'):
	        if transmission.find('joint').attrib['name'] in [joint.attrib['name'] for joint in joints_to_remove]:
	            robot.remove(transmission)
	            
	    for frame in robot.findall('frame'):
	        if frame.attrib['link'] == remove_link:
	            robot.remove(frame)
	    
	    for gazebo in robot.findall('gazebo'):
	        if gazebo.attrib['reference'] == remove_link:
	            robot.remove(gazebo)

# Set the effort limit on a given joint                
def set_joint_effort_limit(robot, name, effort):
    for joint in robot.findall('joint'):
        if joint.attrib['name'] == name:
            joint.find('limit').attrib['effort'] = effort                

if __name__=="__main__":
	atlasFull = ET.parse(os.path.expanduser('~')+'/drake-distro/drake/examples/Atlas/urdf/atlas_minimal_contact.urdf')
	robot = atlasFull.getroot()
	remove_chain(robot, ['ltorso'])

	atlasFull.write('/home/sam/drake-distro/drake/examples/Atlas/urdf/atlas_hips_down.urdf')

