import re
import pymel.core as pm


def getPosition(transform):
    """
    Gets position of move manipulator where control is.

    Args:
        transform (string): name of object to get position from.

    Returns:
        (list): [x, y, z], world position of object in three coordinates.
    """
    pm.select(transform)
    pm.setToolTo('Move')
    pos = pm.manipMoveContext('Move', q=1, p=1)
    pm.select(clear=True)
    return pos


def getAverage(items):
    """
    Gets the average amount in a list.

    Args:
        items (list): must contain floats or integers.

    Returns:
        (integer, float): average of all the items in list.
    """
    return sum(items) / len(items)


def stringIntoList(x):
    """
    Turns a string into a list with the string being the only object in that list if it isnt already a list.

    Args:
        x (Any): to be placed in a list by itself.

    Returns:
        (list): will contain variable 'x' in it all by its lonesome.
    """
    if not x:
        return []

    if not isinstance(x, list):
        x = [x]

    return x


def checkMakeGroup(nodes):
    """
    Makes an empty group if object doesnt exist, if object exist then return PyNode of object.

    Args:
        nodes (string or list): Object(s) to check to see if they exist, if not make them as an empty transform.

    Returns:
        (string or list): PyNode(s) of transforms made.
    """
    nodes = stringIntoList(nodes)
    return_list = []

    for node in nodes:
        if pm.objExists(node):
            grp = pm.PyNode(node)
            return_list.append(grp)
        else:
            grp = pm.group(em=True, n=node)
            return_list.append(grp)

    if len(return_list) == 1:
        return return_list[0]
    else:
        return return_list


def getDigits(word):
    """
    Gets the digits found in a string.

    Args:
        word (string): string to look for digits in.

    Returns:
        (list): list of all digits found in args: word.
    """
    return map(int, re.findall('\d+', word))


def findMaxDigit(words):
    """
    Gets the max digit plus one found at the end of the given words.

    Args:
         words (list): Words to find digits of.

    Returns:
        (string): The max digit from all the words plus one.
    """
    if not words:
        return '0'

    max_digits = [getDigits(word)[-1] for word in words]
    return str(int(max(max_digits)) + 1)


def bakeMotion(controls, start_frame=None, end_frame=None):
    """
    Bakes keyframes onto objects passed as 'controls'

    Args:
        controls (list): transform nodes to have attributes keyed

        start_frame (int): if specified, this is the first frame that will be keyed/baked

        end_frame (int): if specified, this is the last frame that will be keyed/baked
    """
    # get what frames our bake should start and end
    if not start_frame:
        start_frame = pm.playbackOptions(q=1, min=1)
    if not end_frame:
        end_frame = pm.playbackOptions(q=1, max=1)

    pm.select(controls)
    pm.bakeResults(sm=True, time=(start_frame, end_frame))

    # clear the selection
    pm.select(cl=True)


def makeHair(controls, parent_control):
    """
    Makes a hair curve, joints with same orientation as given controls, and constraints the given controls to the joints.
    Uses given parent_control to drive the root of the hair curve.

    Args:
        controls (list): Controls to constrain and build hair curve from.

        parent_control (string or PyNode): Transform that drives hair curve.

    Returns:
        (list): Nodes created for the dynamic hair cuve.
    """
    joints = []
    positions = []
    sum_tx = []
    sum_ty = []
    sum_tz = []
    sum_rx = []
    sum_ry = []
    sum_rz = []
    translate_lock_states = []

    # makes joints at control's position
    for ctrl in controls:

        # Check to make sure rotate attribute of controls are unlocked
        for axis in ['rx', 'ry', 'rz']:
            if pm.getAttr(ctrl.attr(axis), lock=True):
                pm.error('Controls must have rotate attributes unlocked!')

        position = getPosition(ctrl)
        joint = pm.joint(p=position)
        joints.append(joint)
        positions.append(position)

    # makes a new joint at the start of the chain to be used as a way to compensate rotation values
    joints.insert(0, pm.joint(p=getPosition(controls[0])))

    # parent the joints to make a chain and have correct rotation values
    for i in range(len(joints)):

        # parent the joints and set joint orient to 0
        if i != 0:
            pm.parent(joints[i], joints[i - 1])
            joints[i].jo.set(0, 0, 0)

        # if this is the compensation joint, then aim it at the grandchild to get correct rotation values
        if joints[i] == joints[0]:
            pm.aimConstraint(joints[i + 2], joints[i], mo=False, aim=[1, 0, 0])
            pm.delete(pm.listRelatives(joints[i], type="aimConstraint"))

            # aim the joint at the child to get correct rotation values
        elif joints[i] != joints[-1]:
            pm.aimConstraint(joints[i + 1], joints[i], mo=False, aim=[1, 0, 0])
            pm.delete(pm.listRelatives(joints[i], type="aimConstraint"))

    # remove the compensation joint from the joint list
    compensation_joint = joints.pop(0)

    # gets the translate and rotate x,y,z values of each joint and puts them in their own list
    for i, arm in enumerate(joints):
        if joints[i] != joints[0]:
            sum_tx.append(arm.tx.get())
            sum_ty.append(arm.ty.get())
            sum_tz.append(arm.tz.get())

        if joints[i] != joints[-1]:
            sum_rx.append(arm.rx.get())
            sum_ry.append(arm.ry.get())
            sum_rz.append(arm.rz.get())

    # Set the last joint in the chain to have the average rotation value of all the other joints
    joints[-1].r.set(getAverage(sum_rx), getAverage(sum_ry), getAverage(sum_rz))

    # Makes a joint at average sum position of the other joins (end of chain)
    end_joint = pm.joint(n='end_joint')
    pm.parent(end_joint, joints[-1])
    end_joint.t.set(getAverage(sum_tx), getAverage(sum_ty), getAverage(sum_tz))
    joints.append(end_joint)

    pm.parentConstraint(parent_control, joints[0], mo=1)

    # make the curve and look for a hair system, follicles, and time
    ik_handle = pm.ikHandle(sj=joints[0], ee=joints[-1], roc=1, pcv=0, scv=0, snc=1, sol='ikSplineSolver', ccv=1)
    curve = ik_handle[-1]
    pm.delete(ik_handle[0])
    # curve = pm.curve(p=positions, n=cntrls[0] + '_curve', d=len(positions)-1)
    hair_sys = pm.ls(type='hairSystem')
    follicles = pm.ls(type='follicle')
    time = pm.PyNode('time1')

    # if hair system already exists, hook up the curve to that hair system, otherwise make a new one
    if hair_sys:

        # get hair system and nucleus
        hair_sys = hair_sys[0]
        nucleus = pm.ls(type='nucleus')[0]

        # get groupd to keep stuff tidy
        hair_sys_grp = checkMakeGroup('hairSystemOutputCurves')
        hair_follicle_grp = checkMakeGroup('hairSystemFollicles')

        curve_output = curve.duplicate(n=curve + '_output')[0]
        pm.parent(curve_output, hair_sys_grp)

        follicle = pm.createNode('follicle', name=curve + '_follicle' + findMaxDigit(follicles))
        pm.parent(follicle, hair_follicle_grp)
        pm.parent(curve, follicle)
        pm.parentConstraint(parent_control, follicle.getParent(), mo=1)

        follicle.pointLock.set(1)
        follicle.startDirection.set(1)

        # connecting hairSys
        curve.getShape().local >> follicle.startPosition
        curve.worldMatrix >> follicle.startPositionMatrix
        follicle.outCurve >> curve_output.getShape().create

        hair_sys_arrays = hair_sys.inputHair.getArrayIndices()

        evaluators = []
        for i, hair in enumerate(hair_sys_arrays):
            if i == hair:
                evaluators.append(True)
            else:
                follicle.outHair >> hair_sys.inputHair[i]
                hair_sys.outputHair[i] >> follicle.currentPosition
                evaluators.append(False)
                break

        if all(evaluators):
            follicle.outHair >> hair_sys.inputHair[hair_sys_arrays[-1] + 1]
            hair_sys.outputHair[hair_sys_arrays[-1] + 1] >> follicle.currentPosition

    else:
        # This command does not work, so hooking everything up through script
        # pm.mel.eval('MakeCurvesDynamic')

        hair_follicle_grp = pm.group(em=True, n='hairSystemFollicles')
        hair_sys_grp = pm.group(em=True, n='hairSystemOutputCurves')
        nucleus = pm.createNode('nucleus', n='hairNucleus')
        hair_sys = pm.createNode('hairSystem', n='hairSystem')

        curve_output = curve.duplicate(n=curve + '_output')[0]
        pm.parent(curve_output, hair_sys_grp)

        follicle = pm.createNode('follicle', name=curve + '_follicle' + findMaxDigit(follicles))
        pm.parent(follicle, hair_follicle_grp)
        pm.parent(curve, follicle)
        pm.parentConstraint(parent_control, follicle.getParent(), mo=1)

        hair_sys.active.set(1)
        hair_sys.hairsPerClump.set(1)
        hair_sys.selfCollide.set(1)
        follicle.pointLock.set(1)
        follicle.startDirection.set(1)

        # connecting hairSys
        time.outTime >> nucleus.currentTime
        time.outTime >> hair_sys.currentTime
        curve.getShape().local >> follicle.startPosition
        curve.worldMatrix >> follicle.startPositionMatrix
        follicle.outCurve >> curve_output.getShape().create
        follicle.outHair >> hair_sys.inputHair[0]
        hair_sys.startState >> nucleus.inputActiveStart[0]
        hair_sys.currentState >> nucleus.inputActive[0]
        nucleus.startFrame >> hair_sys.startFrame
        nucleus.outputObjects[0] >> hair_sys.nextState
        hair_sys.outputHair[0] >> follicle.currentPosition

    pm.ikHandle(sj=joints[0], ee=joints[-1], roc=0, pcv=0, scv=0, snc=1, curve=curve_output, sol='ikSplineSolver', ccv=0)
    nodes_created = [hair_follicle_grp, hair_sys_grp, nucleus, hair_sys.getParent(), compensation_joint]

    # check to see if translate values of control are locked
    for ctrl in controls:
        translate_lock_state = []
        for axis in ['tx', 'ty', 'tz']:
            translate_lock_state.append(pm.getAttr(ctrl.attr(axis), k=True))
        translate_lock_states.append(translate_lock_state)

    # attach the hair curve to the controls and add the constraints to the to delete list
    for i, node in enumerate(zip(joints, controls)):
        if not any(translate_lock_states[i]):
            const = pm.orientConstraint(node[0], node[1], mo=1)
            nodes_created.append(const)
        else:
            const = pm.parentConstraint(node[0], node[1], mo=1)
            nodes_created.append(const)

    pm.select(cl=True)
    return nodes_created


def cleanUp(garbage):
    """
    Convenience method for deleting a list of nodes.

    Args:
        garbage (list): Nodes to delete.
    """
    for node in garbage:
        if pm.objExists(node):
            pm.delete(node)


class HairTool(object):
    """
    GUI for making a hair curve drive controls.
    """
    def __init__(self):
        self.window_name = 'HairTool'
        self.controls = []
        self.parent_control = ''
        self.created_nodes = []
        self.control_scroll_field = None
        self.parent_scroll_field = None
        self.build()

    def build(self):
        """
        Creates the GUI.
        """
        # if window exists, delete it
        if pm.window(self.window_name, exists=True):
            pm.deleteUI(self.window_name)

        with pm.window(self.window_name, rtf=True, menuBar=True, menuBarVisible=True, title='HairTool') as hairTool_window:
            with pm.verticalLayout():
                with pm.frameLayout(label='Make Hair Curve', collapsable=True):
                    # assign controls field
                    pm.text(label='Control List:', align='left')
                    self.control_scroll_field = pm.scrollField(w=300, h=35, wordWrap=1, ed=0, en=0, tx="Select control chain in order then press Assign Controls.")
                    pm.button(l='Assign Controls', c=self._assignControls)

                    # Assign parent field
                    pm.separator(h=15, w=300, style="in")
                    pm.text(l='Parent', al='left')
                    self.parent_scroll_field = pm.scrollField(w=300, h=35, wordWrap=1, ed=0, en=0, tx='Select the parent control and then press Assign Parent')
                    pm.button(l='Assign Parent', c=self._assignParent)

                    pm.separator(h=15, w=300, style="in")
                    pm.button(l='Create Hair Curve', c=pm.Callback(self._makeHair))
                    pm.separator(h=15, w=300, style="in")

                with pm.frameLayout(label='Bake Curve Motion', collapsable=True, h=20):
                    with pm.horizontalLayout():
                        pm.button(l='Bake Motion', c=pm.Callback(self._bakeClean, clean=True), h=10)
                        pm.button(l='Bake and keep nodes', c=pm.Callback(self._bakeClean, clean=False), h=10)

    def _assignControls(self, *args):
        """
        Assign controls in class and show them in the GUI.
        """
        control_name = ''
        self.controls = pm.selected()
        for i, control in enumerate(self.controls):
            if self.controls[i] != self.controls[-1]:
                control_name = control_name + str(control) + ', '
            else:
                control_name = control_name + str(control)
        pm.scrollField(self.control_scroll_field, e=True, tx=control_name)

    def _assignParent(self, *args):
        """
        Assign the parent control in class and show it in the GUI.
        """
        self.parent_control = pm.selected()[0]
        pm.scrollField(self.parent_scroll_field, e=True, tx=self.parent_control)

    def _makeHair(self, *args):
        """
        Stores nodes to be deleted after making the hair curve/system.
        """
        self.created_nodes = makeHair(self.controls, self.parent_control)

    def _bakeClean(self, clean=True, *args):
        """
        Bakes animation based timeline's playback range on and deletes hair system nodes if given clean is True.

        Args:
            clean (boolean): If True, will delete all hair system nodes made
        """
        bakeMotion(self.controls)
        if clean:
            cleanUp(self.created_nodes)
