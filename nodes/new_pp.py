#!/usr/bin/env python3

# importacao das bibliotecas padrão do Python
from copy import deepcopy
import sys
import math
import time

# importacao ROS
import rospy

# importacao MoveIt e TF
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander
import tf.transformations as tf

#usada para gerar as poses
import random

# importacao msgs and srvs
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_geometry_msgs import tf2_geometry_msgs
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, Grasp, PlaceLocation
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Quaternion, Pose, Vector3
import geometry_msgs.msg as gm

# classe que define um objeto posicao
class Position:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z

# constroi um vetor de posicoes
target_object_positions = []
CURRENT_POSITION_INDEX = 0

#parametros que sao usados nas consfiguracoes de pre e pos grasp
#dimensoes do objeto alvo
DIMENSOES_CUBO = Vector3(0.07,0.07,0.07)
POSICAO_CUBO = Position(0.0, 0.5, 0.3)
#posicao que o objeto será colocado
POSICAO_MESA2 = Position(0,-0.5,0.136434)
DIMENSOES_MESA2 = Vector3(0.399110, 0.399110, 0.272868)

#funcao que gera um numero escolhido de poses, dada uma posicao conhecida 
#de um objeto
def generate_poses(num_poses:int,basepose:Pose) -> list:
    poses = []
    VARIATION = 0.002
    for _ in range(num_poses):
        # Definir coordenadas x, y, z
        pose = deepcopy(basepose)
        # Intervalo de coordenadas x da mesa
        pose.position.x += random.uniform(-VARIATION, VARIATION) 
        # Intervalo de coordenadas y da mesa
        pose.position.y += random.uniform(-VARIATION, VARIATION) 
        
        poses.append(pose)
    
    return poses


# carrega todas posicoes a serem testadas dentro do vetor
def load_all_positions():
  target_object_positions.append(POSICAO_CUBO) 

# constante com o nome do objeto que sera manipulado
OBJECT_TARGET_NAME = "box"

# metodo que e responsavel por abrir a garra (abre o dedo1 e o dedo2)
def open_gripper(posture):
    """
    - Abrir a garra -
    Parametros: Posicao da Garra
        posture : trajectory_msgs.msg.JointTrajectory
    """

    # adiciona as duas juntas dos dedos da garra do panda
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    # seta os dedos para abrir com uma largura suficiente para caber o objeto desejado
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = 0.04
    posture.points[0].positions[1] = 0.04
    posture.points[0].time_from_start = rospy.Duration(0.5)

# metodo responsavel por fechar a garra (fecha o dedo1 e o dedo2)
def close_gripper(posture):
    """
    - Fechar a garra -
    Parametros: Posicao da Garra
        posture : trajectory_msgs.msg.JointTrajectory
    """

    # adiciona as duas juntas dos dedos da garra do panda
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    # seta os dedos para fechar
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = 0.0345
    posture.points[0].positions[1] = 0.0345
    posture.points[0].time_from_start = rospy.Duration(0.5)

# metodo responsavel por pegar o objeto
def pick(move_group : MoveGroupCommander):
    """
    - Pegar o objeto -
    Parametros: 
    ----------
    Group : moveit_commander.RobotCommander
                    Moveit_commander move group.
    """

    NUMERO_PEGADAS = 10
    DISTANCIA_SEGURANCA = 0.03
    DISTANCIA_LINK_EFETOR = 0.058
    hand_pose = move_group.get_current_pose("panda_link7").pose
    ref_pose = Pose()
    # Usa a posição do cubo como referencia
    ref_pose.position = deepcopy(POSICAO_CUBO)
    ref_pose.position.z += DIMENSOES_CUBO.z/2 + DISTANCIA_SEGURANCA + DISTANCIA_LINK_EFETOR
    # Usa da orientação original da mão
    ref_pose.orientation = hand_pose.orientation

    #chama a funcao geradora de poses
    poselist = generate_poses(NUMERO_PEGADAS,ref_pose)
    # cria um vetor do tipo Grasp() de pegadas a serem tentadas
    # variando somente o x e o y, setando a direcao de pegada como "-z"
    grasps = [Grasp() for i in range(NUMERO_PEGADAS)]

    for i,grasp in enumerate(grasps):

        grasp.grasp_pose.header.frame_id = "panda_link0"
        grasp.grasp_pose.pose = poselist[i]

        ## Setando a aplicacao de pre pegada
        # definindo com relação ao frame_id
        grasp.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        # a direcao e setada como eixo z negativo (pega o cubo por cima)
        grasp.pre_grasp_approach.direction.vector.z = -1.0
        grasp.pre_grasp_approach.min_distance = 0.06
        grasp.pre_grasp_approach.desired_distance = 0.07

        ## Setando o recuo pos pegada
        # definindo com relação ao frame_id
        grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        # a direcao e setada como eixo z positivo
        grasp.post_grasp_retreat.direction.vector.z = 1.0
        grasp.post_grasp_retreat.min_distance = 0.05
        grasp.post_grasp_retreat.desired_distance = 0.08

        ## Setando a postura do efetor final antes da pegada
        open_gripper(grasp.pre_grasp_posture)

        ## Setando a postura do efetor final durante a pegada
        close_gripper(grasp.grasp_posture)

    ## Setando a superfície de suporte como table1
    move_group.set_support_surface_name("table1")

    ## Chama o metodo pick para pegar o objeto usando as pegadas fornecidas como parametro
    move_group.pick(OBJECT_TARGET_NAME, grasps)

def place(group):
    """
    - Coloca o objeto -
    Parametros: 
    ----------
    Group : moveit_commander.RobotCommander
                    Moveit_commander move group
                
    """
    DISTANCIA_SEGURANCA = 0.0005
    DISTANCIA_LINK_EFETOR = 0.028
     # Usa a posição da mesa 2 como referencia para o place
    ref_pose = Pose()
    base_pose = move_group.get_current_pose("panda_link0").pose
    ref_pose.position = deepcopy(POSICAO_MESA2)
    ref_pose.position.z += DIMENSOES_MESA2.z/2 +DIMENSOES_CUBO.z/2+ DISTANCIA_SEGURANCA
   
    # Usa da orientação original da mão
    ref_pose.orientation = deepcopy(base_pose.orientation)
    #vetor de lugares para tentar colocar o objeto manipulado na mesa 2
    #em uma posicao já conhecida (não tenta várias posições)
    place_location = [PlaceLocation() for i in range(1)]

    
    # defindo a pose para se colocar o objeto, onde e como colocá-lo
    place_location[0].place_pose.header.frame_id = "panda_link0"
    place_location[0].place_pose.pose = ref_pose
    

    # estabelecendo a abordagem pre-place em relação ao frame_id com a direção em z negativa 
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0"
    place_location[0].pre_place_approach.direction.vector.z = -1.0
    place_location[0].pre_place_approach.min_distance = 0.095
    place_location[0].pre_place_approach.desired_distance = 0.115

    # estabelecendo o recuo post-grasp em relaçaõ ao frame_id
    # direção x é negativa
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0"
    place_location[0].post_place_retreat.direction.vector.z = 1.0
    place_location[0].post_place_retreat.min_distance = 0.1
    place_location[0].post_place_retreat.desired_distance = 0.25

    ## Setando a postura do efetor final apos colocar o objeto
    open_gripper(place_location[0].post_place_posture)
 
    ## Estabelece suporte para a superfície da mesa 2, i.e, a mesa que será colocada o objeto manipulado
    group.set_support_surface_name("table2")

    ## Chama o place para colocar o objeto com a localização dada
    group.place(OBJECT_TARGET_NAME, place_location[0])


def add_collision_objects(planning_scene_interface):

    # Aqui será criado os três objetos que estão no cenário. Duas "mesas" e um "cubo" que será 
    # o objeto que se deseja mover
    collision_objects_names = [str for i in range(3)]
    collision_object_sizes = [str for i in range(3)]
    collision_objects = [PoseStamped() for i in range(3)]

    ## Esse trecho de codigo e responsavel por setar nome, frame_id, tamanho e posicao de um objeto no cenário
    # adiciona a segunda mesa
    collision_objects_names[0] = "table2"
    collision_objects[0].header.frame_id = "panda_link0"
    # define as dimensões da segunda mesa
    collision_object_sizes[0] = (0.399110, 0.399110, 0.272868)  
    # define a pose da mesa
    collision_objects[0].pose.position.x = 0.0
    collision_objects[0].pose.position.y = -0.5
    collision_objects[0].pose.position.z = 0.136434

    ##########################################################
    # aqui será repetido o processo acima para inserir no cenário a primeira mesa
    # que o cubo está em cima
    collision_objects_names[1] = "table1"
    collision_objects[1].header.frame_id = "panda_link0"
    collision_object_sizes[1] = (0.399110, 0.399110, 0.272871)   
    collision_objects[1].pose.position.x = 0.0
    collision_objects[1].pose.position.y = 0.5
    collision_objects[1].pose.position.z = 0.136434
    ##########################################################


    ## Inserindo o objeto que vai ser manipulado pelo braço no cenário
    collision_objects_names[2] = OBJECT_TARGET_NAME
    collision_objects[2].header.frame_id = "panda_link0"
    # define as dimensões dele
    collision_object_sizes[2] = (0.069404, 0.069404, 0.069404)  # Box size
    # define a pose
    collision_objects[2].pose.position.x = target_object_positions[CURRENT_POSITION_INDEX].x
    collision_objects[2].pose.position.y = target_object_positions[CURRENT_POSITION_INDEX].y
    collision_objects[2].pose.position.z = target_object_positions[CURRENT_POSITION_INDEX].z

    # Adicionando todos os objetos no cenário no Rviz
    for (name, pose, size) in zip(
            collision_objects_names, collision_objects, collision_object_sizes
    ):
        planning_scene_interface.add_box(name=name, pose=pose, size=size)


if __name__ == "__main__":

    # inicializa o no do ROS
    rospy.init_node("pick_and_place_using_panda_arm")

    # carrega o vetor de posicoes possiveis para o objeto alvo que será deslocado
    load_all_positions()

    # inicializa o moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # conecta moveit services
    rospy.loginfo(
        "Conneting moveit default moveit 'apply_planning_scene' service.")
    rospy.wait_for_service("apply_planning_scene")
    try:
        planning_scene_srv = rospy.ServiceProxy(
            "apply_planning_scene", ApplyPlanningScene
        )
        rospy.loginfo("Moveit 'apply_planning_scene' service found!")
    except rospy.ServiceException as e:
        rospy.logerr(
            "Moveit 'apply_planning_scene' service initialization failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            planning_scene_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    # cria um robot commander
    robot = RobotCommander(
        robot_description="robot_description", ns="/"
    )
    rospy.logdebug("Robot Groups: %s", robot.get_group_names())
    print("Robot Groups: %s", robot.get_group_names())

    # pegando informação sobre o mundo e atualizando o entendimento que o robo tem do mundo
    move_group = robot.get_group("panda_arm")
    planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")

    rospy.sleep(1.0)
    planning_scene_interface.remove_world_object()  # Does not work at the beginning
    rospy.sleep(1.0)

    # especificando o planejador de trajetorias que queremos usar
    move_group.set_planner_id("TRRTkConfigDefault")
    robot.get_group("panda_arm").set_goal_position_tolerance(0.01)
    robot.get_group("panda_hand").set_goal_position_tolerance(0.01)
    robot.get_group("panda_manipulator").set_goal_position_tolerance(0.01)


    # cria um DisplayTrajectory que é um no publicador do ROS para mostrar o planejamento no RViz
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )

    # espera um pouco para o ROS incializar o planning_scene_interface
    rospy.sleep(1.0)

    # adiciona os objetos de colisao
    add_collision_objects(planning_scene_interface)
    
    # espera um pouco para carregar os objetos
    rospy.sleep(1.0)

    #iniciando a contagem de tempo
    start_time = time.time()

    # pega o objeto ja pre definido passando como argumento o move_group do panda_arm
    pick(move_group)

    # espera 1seg
    delay = 1.0
    rospy.sleep(delay)

    # coloca o objeto 
    place(move_group)

    #move_group.set_named_target('ready')

    # imprimindo o tempo que levou a execucao menos o delay forçado de 1 segundo
    print("position %s: x=%s, y=%s, z=%s | --- %s seconds ---" % (
      CURRENT_POSITION_INDEX,
      target_object_positions[CURRENT_POSITION_INDEX].x,
      target_object_positions[CURRENT_POSITION_INDEX].y,
      target_object_positions[CURRENT_POSITION_INDEX].z,
      time.time() - start_time - delay)
    )

