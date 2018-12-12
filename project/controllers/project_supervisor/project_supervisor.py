import load_configuration
import random as rnd
from controller import Supervisor, Robot, Node, Field
# Above imported "controller" package is from Webots.


class SupervisorControls(Supervisor):
    def get_node(self, node_def_name):
        return self.getFromDef(node_def_name)

    def step_sim(self):
        self.step(16)

    def get_sim_time(self):
        return self.getTime()

class SimController(object):
    """SimController class controls the flow of the simulation."""
    def __init__(self):
        self.supControl = SupervisorControls()

    def run(self):
        self.identifiers = load_configuration.load_identifiers()
        node1 = self.supControl.get_node('VIEWPOINT')
        for (idx, identifier) in enumerate(self.identifiers):
            name = 'IDENTIFIER_' + str(idx+1)
            node = self.supControl.get_node(name)
            #print('ID: {}, node: {} color: {} prob {} color2 {} prob2 {} color3 {} prob3 {}'.format(idx, name, identifier[0][0], identifier[0][1], identifier[1][0], identifier[1][1], identifier[2][0], identifier[2][1]))
            
            random_number = rnd.random()
            color_prob1 = identifier[0]
            color_prob2 = identifier[1]
            color_prob3 = identifier[2]
            if random_number < color_prob1[1]:
                node_color = color_prob1[0]
                # print('Selected1 color: {}'.format(node_color))
            elif random_number < color_prob1[1] + color_prob2[1]:
                node_color = color_prob2[0]
                # print('Selected2 color: {}'.format(node_color))
            else:
                node_color = color_prob3[0]
                # print('Selected3 color: {}'.format(node_color))
            node_color = node_color.tolist()
            node_color[0] = max(0.0, min(1.0, node_color[0]))
            node_color[1] = max(0.0, min(1.0, node_color[1]))
            node_color[2] = max(0.0, min(1.0, node_color[2]))
            # print('Selected color: {}'.format(node_color))
            field = node.getField('diffuseColor')
            # print('node : {}'.format(name))
            field.setSFColor(node_color)
            field = node.getField('specularColor')
            field.setSFColor(node_color)
            field = node.getField('emissiveColor')
            field.setSFColor([node_color[0]/5.0, node_color[1]/5.0, node_color[2]/5.0])

        current_time = 0

        while self.supControl.step_sim() != -1:
            current_time = self.supControl.get_sim_time()


supervisorcontrol = SimController()
supervisorcontrol.run()
