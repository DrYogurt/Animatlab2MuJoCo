import xmltodict
import xml.etree.ElementTree as et
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

def process_rb(rb):
    new_rb = {}
    keys = ["Name","ID","Mass","Density","Position","Rotation","Type",
            "IsCollisonObject","IsContactSensor",
            "StimulusTension","Length Tension","MaximumTension","Kse","Kpe","B","Attachments"]
    for key,value in rb.items():
        if key in keys:
            new_rb[key] = process_item(key,value)
    
    if "ChildBodies" in rb.keys():
        new_rb["ChildBodies"] = [process_rb(child) for child in rb["ChildBodies"]["RigidBody"]]
    if "Width" in rb.keys(): #if width exists, assume the rest do...
        new_rb["size"] = "{} {} {}".format(float(rb["Length"])*50,float(rb["Height"])*50,float(rb["Width"])*50)
    if "Joint" in rb.keys():
        if isinstance(rb["Joint"],list):
            new_rb["Joints"] = [process_joint(child) for child in rb["Joint"]]
        else:
            new_rb["Joint"] = process_joint(rb["Joint"])
    
    return new_rb

def process_joint(joint):
    new_jnt = {}
    keys = ["Name","Position","Rotation","Type","Size",
            "MaxForce","MaxVelocity","EnableMotor","MotorType","ServoGain"]
    relaxations = []
    for key,value in joint.items():
        if key in keys:
            new_jnt[key] = process_item(key,value,is_jnt=True)
        if key in ["LowerLimit","UpperLimit"]:
            new_jnt[key] = {}
            new_jnt[key]["LimitPos"] = float(value["LimitPos"])
            new_jnt[key]["Damping"] = float(value["Damping"])
            new_jnt[key]["Restitution"] = float(value["Restitution"])
            new_jnt[key]["Stiffness"] = float(value["Stiffness"]) 
        if key == "Friction":
            if value["Enabled"] == True:
                new_jnt["Friction"] = {}
                new_jnt["Friction"]["Type"] = float(value["Type"])
                new_jnt["Friction"]["Coefficient"] = float(value["Coefficient"])
                new_jnt["Friction"]["MaxForce"] = float(value["MaxForce"])
                new_jnt["Friction"]["Loss"] = float(value["Loss"])
                
            
    return new_jnt

def process_item(key,param,is_jnt=False):
    if key =="Position":
        return "{} {} {}".format(float(param["@x"])*100,float(param["@y"])*100,float(param["@z"])*100)
    elif key =="Rotation":
        x0 = float(param["@x"])*180 / math.pi
        y0 = float(param["@y"])*180 / math.pi
        z0 = float(param["@z"])*180 / math.pi
        
        if is_jnt:
            r = R.from_euler("xyz",[x0,-z0,y0],degrees=True)
            r_mat = r.as_matrix()
            [x, y, z] =  np.matmul(r_mat,np.array([1, 0, 0]).transpose())
            print(x,y,z)
            return "{} {} {}".format(x,y,z)
        else:
            return "{} {} {}".format(x0,y0,z0)
    else:
        try:
            float(param)
            return float(param)
        except:
            return param

def xmlify_body(dic,attach_ids={},root=0):
    #print(dic["Name"])
    #print(dic["Type"])
    muscles = []
    body = et.Element("body",attrib={
        "name":dic["Name"],
        "pos":dic["Position"],
        "euler":dic["Rotation"]
    })
    body.append(et.Element("geom",attrib={
        "type": "box",
        "size": dic["size"]
    }))
    if "ChildBodies" in dic.keys():
        for value in dic["ChildBodies"]:
            if value["Type"] == "Attachment":
                body.append(et.Element("site",attrib={
                    "name":value["Name"],
                    "pos":value["Position"]
                }))
                attach_ids[value["ID"]] = value["Name"]
            if value["Type"].__contains__("Muscle"):
                muscles.append(value)
            if value["Type"] == "Box":
                child, child_muscles, _ = xmlify_body(value,attach_ids=attach_ids,root=1)
                body.append(child)
                muscles.append(child_muscles)
    if "Joint" in dic.keys():
            joints = []
            if isinstance(dic["Joint"],list):
                joints = dic["Joint"]
            else:
                joints.append(dic["Joint"])
            for value in joints:
                body.append(et.Element("joint",attrib={
                    "name":value["Name"],
                    "pos":value["Position"],
                    "axis":value["Rotation"] # TODO: Potential issue n+2
                }))

    if root == 0:
        pass
        #for muscle in muscles:
    return body, muscles, attach_ids

def main():
    # Import rat xml and convert to dictonary
    with open("xmls/Whole_leg_sim_Standalone.asim",'r') as f:
        dicto = xmltodict.parse(f.read())
    
    rat = dicto['Simulation']['Environment']["Organisms"]["Organism"]

    dic = process_rb(rat["RigidBody"])
    global_rot = dic["Rotation"].split(" ")
    global_rot[0] = str(float(global_rot[0]) + 90)
    dic["Rotation"] = " ".join(global_rot)
    
    root = et.Element("mujoco")
    root.append(et.Element("compiler",attrib={"coordinate":"local","angle":"degree","eulerseq":"xyz"}))

    default = et.SubElement(root,"default")
    default.append(et.Element("geom",attrib={"rgba":".8 .4 .6 1"}))

    asset = et.SubElement(root,"asset")
    asset.append(et.Element("texture",attrib={"type":"skybox",
                                            "builtin":"gradient",
                                            "rgb1":"1 1 1","rgb2":".6 .8 1",
                                            "width":"256",
                                            "height":"256"
                                            }))
    world_body = et.SubElement(root,"worldbody")
    world_body.append(et.Element("geom",attrib={"name":"floor",
                                                "pos":"0 0 -5",
                                                "size":"10 10 0.125",
                                                "type":"plane",
                                                "condim":"3",
                                                "rgba":"1 1 1 1"
                                            }))
    world_body.append(et.Element("light",attrib={"pos":"0 5 5",
                                                "dir":"0 -1 -1",
                                                "diffuse":"1 1 1"
    }))
    body,muscles,attach_ids = xmlify_body(dic)
    world_body.append(body)
    with open("xmls/NewRat.xml","wb") as fp: 
        et.indent(root)
        tree = et.ElementTree(root)
        tree.write(fp)

if __name__ =="__main__":
    main()