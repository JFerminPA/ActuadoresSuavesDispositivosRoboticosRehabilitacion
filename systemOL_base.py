# -*- coding: utf-8 -*-

"""systemOL_base.py: Implementación de un sistema de control en lazo abierto para un modelo de dedo robótico."""

# In[1]:
# CELDA 1: Importar las librerías necesarias
import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# In[2]:
# CELDA 2: Conectar a PyBullet en modo GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# In[3]:
# CELDA 3: Reiniciar la simulación y configurar parámetros
p.resetSimulation()
p.setRealTimeSimulation(0)
p.setGravity(0, 0, 0)  
time_step = 1./600.    
p.setTimeStep(time_step)
p.setPhysicsEngineParameter(numSolverIterations=80)
p.setPhysicsEngineParameter(numSubSteps=5)

# In[4]:
# CELDA 4: Cargar el modelo URDF
finger = p.loadURDF("fingerBox4DoFwC4.urdf", basePosition=[0, -0.01, 0], useFixedBase=True)

# In[5]:
# CELDA 5: Comprobación exhaustiva del robot 'finger'

numLinks = 4
linkLengths = [0.005, 0.03, 0.03, 0.02]

num_joints = p.getNumJoints(finger)
print(f"Número de articulaciones en el cuerpo multienlace 'finger': {num_joints}")
if num_joints != numLinks:
    print(f"Error: Se esperaban {numLinks} articulaciones, pero se encontraron {num_joints}.")
else:
    print("Las articulaciones se han creado correctamente.")

link_names = ["proximal_phalanx_base", "proximal_phalanx", "middle_phalanx", "distal_phalanx"]
for linkIndex in range(numLinks):
    link_state = p.getLinkState(finger, linkIndex)
    print(f"Falange {linkIndex} ({link_names[linkIndex]}) Position: {link_state[0]}")
    print(f"Falange {linkIndex} ({link_names[linkIndex]}) Orientation: {link_state[1]}")

base_pos, base_orn = p.getBasePositionAndOrientation(finger)
print(f"Base Position: {base_pos}")
print(f"Base Orientation: {base_orn}")

joint_names = ["MCP_AbAd", "MCP_FlexExt", "PIP", "DIP"]
for jointIndex in range(num_joints):
    joint_info = p.getJointInfo(finger, jointIndex)
    joint_id = joint_info[0]
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    parent_index = joint_info[16]
    child_index = joint_info[0]
    print(f"Articulación {jointIndex}:")
    print(f"  Nombre: {joint_name}")
    print(f"  Tipo: {joint_type}")
    print(f"  Índice del Padre: {parent_index}")
    print(f"  Índice del Hijo: {child_index}")
    print(f"  Límite Inferior: {joint_info[8]}")
    print(f"  Límite Superior: {joint_info[9]}\n")

for jointIndex in range(numLinks):
    info = p.getJointInfo(finger, jointIndex)
    current_mode = p.getDynamicsInfo(finger, jointIndex)
    print(f"\nArticulación {jointIndex}:")
    print(f"  Modo de control actual: {current_mode}")

hierarchy = {}
for jointIndex in range(num_joints):
    joint_info = p.getJointInfo(finger, jointIndex)
    joint_name = joint_info[1].decode('utf-8')
    parent_index = joint_info[16]
    child_index = joint_info[0]
    hierarchy[child_index] = {
        'name': joint_name,
        'parent': parent_index
    }

def print_hierarchy(link_index, level=0):
    indent = "  " * level
    if link_index in hierarchy:
        print(f"{indent}Link {link_index}: {hierarchy[link_index]['name']}")
        for child, info in hierarchy.items():
            if info['parent'] == link_index:
                print_hierarchy(child, level + 1)

print("Jerarquía del MultiBody 'finger':")
print("Base")
for linkIndex in range(num_joints):
    if hierarchy[linkIndex]['parent'] == -1:
        print_hierarchy(linkIndex, level=1)

# In[6]:
for linkIndex in range(numLinks):
    p.setCollisionFilterPair(finger, finger, -1, linkIndex, enableCollision=1)
for linkIndex in range(numLinks - 1):
    p.setCollisionFilterPair(finger, finger, linkIndex, linkIndex + 1, enableCollision=1)

# In[7]:
for jointIndex in range(numLinks):
    p.setJointMotorControl2(
        bodyIndex=finger,
        jointIndex=jointIndex,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=0
    )

# In[8]:
frictionAnchor = [0, 1, 1, 1]

def variar_parametros(parametros, porcentaje=0.20):
    parametros_variados = []
    for param in parametros:
        variacion = np.random.uniform(-porcentaje, porcentaje)
        param_variado = param * (1 + variacion)
        parametros_variados.append(param_variado)
    return parametros_variados

jointDampings_base = [0., 0.00027, 0.00028, 0.00045]
linearDampings_base = [0., 1.60, 1.51, 1.51]
angularDampings_base = [0., 1.7, 1.7, 1.7]
contactDamping_base = [0., 2.9, 1.93, 0.93]
contactStiffness_base = [0., 1.9, 1.73, 2.93]

k1_springs_initial_base = [0., 0.030, 0.013, 0.0018]
k1_springs_final_base = [0., 0.0001, 0.0001, 0.0001]

k_dampings_initial_base = [0., 0.00046, 0.00066, 0.00002]
k_dampings_final_base = [0., 0.000016, 0.000016, 0.000012]

jointDampings = variar_parametros(jointDampings_base, porcentaje=0.20)
linearDampings = variar_parametros(linearDampings_base, porcentaje=0.20)
angularDampings = variar_parametros(angularDampings_base, porcentaje=0.20)
contactDamping = variar_parametros(contactDamping_base, porcentaje=0.20)
contactStiffness = variar_parametros(contactStiffness_base, porcentaje=0.20)

k1_springs_initial = variar_parametros(k1_springs_initial_base, porcentaje=0.20)
k1_springs_final = variar_parametros(k1_springs_final_base, porcentaje=0.20)

k_dampings_initial = variar_parametros(k_dampings_initial_base, porcentaje=0.20)
k_dampings_final = variar_parametros(k_dampings_final_base, porcentaje=0.20)


for jointIndex in range(numLinks):
    p.changeDynamics(
        bodyUniqueId=finger,
        linkIndex=jointIndex,
        jointDamping=jointDampings[jointIndex],
        linearDamping=linearDampings[jointIndex],
        angularDamping=angularDampings[jointIndex],
        contactDamping=contactDamping[jointIndex],
        contactStiffness=contactStiffness[jointIndex],
        frictionAnchor=frictionAnchor[jointIndex],
        restitution=0
    )

# In[9]:
sphere_radius = 0.002
sphere_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=sphere_radius,
    rgbaColor=[1, 0, 0, 1]
)

for jointIndex in range(numLinks):
    joint_pos = p.getLinkState(finger, jointIndex, computeForwardKinematics=True)[4]
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=sphere_visual_shape_id,
        basePosition=joint_pos,
        useMaximalCoordinates=True
    )

# In[10]:
p.resetDebugVisualizerCamera(
    cameraDistance=0.11,
    cameraYaw=84,
    cameraPitch=-0.4,
    cameraTargetPosition=[0, 0.05, 0]
)

# In[11]:
total_steps_flexion = 2000
total_steps_extension = 2000
total_steps_total = total_steps_flexion + total_steps_extension

Torque_max_flexion =  [0.0,  -0.039, -0.021, -0.019]
Torque_max_extension = [0.0, +0.034, +0.023, +0.014]

start_steps_flexion = [1000, 80, 221, 340]
end_steps_flexion = [total_steps_flexion] * numLinks

start_steps_extension = [1000, 75, 211, 361]
end_steps_extension = [total_steps_extension] * numLinks

final_flexion_angles = np.zeros(numLinks)

# In[12]:
def get_torque_magnitude(step, start_step, end_step, max_torque, 
                         total_steps_flexion, total_steps_extension, 
                         exp_flexion, exp_extension):
    """
    Calcula el torque a aplicar en función del paso (step) usando un crecimiento
    basado en el exponente exp_flexion durante la fase de flexión y en exp_extension
    durante la fase de extensión.
    """
    total_steps_total = total_steps_flexion + total_steps_extension
    if step < total_steps_flexion:
        # Fase de flexión: uso de crecimiento según exp_flexion
        if step < start_step:
            return 0.0
        elif step > end_step:
            return max_torque
        else:
            ratio = (step - start_step) / (end_step - start_step)
            return max_torque * (ratio ** exp_flexion)
    elif step < total_steps_total:
        # Fase de extensión: uso de crecimiento según exp_extension
        extension_step = step - total_steps_flexion
        if extension_step < start_step:
            return 0.0
        elif extension_step > end_step:
            return max_torque
        else:
            ratio = (extension_step - start_step) / (end_step - start_step)
            return max_torque * (ratio ** exp_extension)
    else:
        return 0.0


# In[13]:
def apply_external_torques(step):
    torques = [0.0] * numLinks
    if step < total_steps_flexion:
        Torque_max = Torque_max_flexion
        start_steps = start_steps_flexion
        end_steps = end_steps_flexion
        fase = "Flexión"
    elif step < total_steps_total:
        Torque_max = Torque_max_extension
        start_steps = start_steps_extension
        end_steps = end_steps_extension
        fase = "Extensión"
    else:
        return torques

    exp_flexion_list = [None, 1.82, 2.41, 2.16]     
    exp_extension_list = [None, 1.82, 2.16, 2.67]     

    for i in range(numLinks):        
        if i == 0:
            torques[i] = 0.0
            continue
        
        if fase == "Flexión":
            exp_val_flexion = exp_flexion_list[i]            
            torque = get_torque_magnitude(
                step, 
                start_steps[i], 
                end_steps[i], 
                Torque_max[i],
                total_steps_flexion, 
                total_steps_extension,
                exp_val_flexion,  
                0  
            )
        else:  # Fase de Extensión
            exp_val_extension = exp_extension_list[i]
            torque = get_torque_magnitude(
                step, 
                start_steps[i], 
                end_steps[i], 
                Torque_max[i],
                total_steps_flexion, 
                total_steps_extension,
                0,  
                exp_val_extension  
            )

        print(f"Paso {step}, {fase}, Articulación {i}: Torque aplicado = {torque:.5f} Nm")

        if torque != 0.0:
            torque_vector = [torque, 0, 0]
            p.applyExternalTorque(
                objectUniqueId=finger,
                linkIndex=i,
                torqueObj=torque_vector,
                flags=p.WORLD_FRAME
            )
        torques[i] = torque
    return torques


# In[14]:
def apply_joint_torques(step):
    if step < total_steps_flexion:
        proportion = step / total_steps_flexion
        k1_springs = [initial * (1 - proportion) + final * proportion
                     for initial, final in zip(k1_springs_initial, k1_springs_final)]
        k_dampings = [initial * (1 - proportion) + final * proportion
                     for initial, final in zip(k_dampings_initial, k_dampings_final)]
    elif step < total_steps_total:
        extension_step = step - total_steps_flexion
        proportion = extension_step / total_steps_extension
        k1_springs = [final * (1 - proportion) + initial * proportion
                     for initial, final in zip(k1_springs_initial, k1_springs_final)]
        k_dampings = [final * (1 - proportion) + initial * proportion
                     for initial, final in zip(k_dampings_initial, k_dampings_final)]
    else:
        k1_springs = [0.0] * numLinks
        k_dampings = [0.0] * numLinks

    for jointIndex in range(numLinks):
        if jointIndex == 0:
            continue
        jointState = p.getJointState(finger, jointIndex)
        jointPosition = jointState[0]
        jointVelocity = jointState[1]       
        
        adjusted_position = jointPosition - final_flexion_angles[jointIndex]

        torque_spring = - (k1_springs[jointIndex]) * adjusted_position
        torque_damping = - (k_dampings[jointIndex]) * (jointVelocity)
        torque_sd = torque_spring + torque_damping

        if torque_sd != 0.0:
            torque_vector = [torque_sd, 0, 0]
            p.applyExternalTorque(
                objectUniqueId=finger,
                linkIndex=jointIndex,
                torqueObj=torque_vector,
                flags=p.WORLD_FRAME
            )

        print(f"Articulación {jointIndex}:")
        print(f"  Posición (deg) = {np.degrees(jointPosition):.2f}°, Velocidad (deg/s) = {np.degrees(jointVelocity):.2f}°/s")
        print(f"  Torque Spring = {torque_spring:.4f} Nm, Torque Damping = {torque_damping:.4f} Nm")
        print(f"  Torque Total Aplicado = {torque_sd:.4f} Nm")

# In[15]:
times = []
jointAngles = [[] for _ in range(numLinks)]
appliedTorques = [[] for _ in range(numLinks)]

# In[16]:
for jointIndex in range(numLinks):
    p.enableJointForceTorqueSensor(finger, jointIndex, 1)

# In[17]:
# CELDA 17: Actualización del Bucle de Simulación    

for step in range(total_steps_total):
    torques_applied = apply_external_torques(step)
    apply_joint_torques(step)

    p.stepSimulation()
    time.sleep(time_step)

    times.append(step * time_step)
    print(f"\nPaso {step}:")
    for jointIndex in range(numLinks):
        state = p.getJointState(finger, jointIndex)
        jointAngle = state[0]
        jointVelocity = state[1]
        jointTorque = state[3]
        jointAngles[jointIndex].append(np.degrees(jointAngle))
        appliedTorques[jointIndex].append(torques_applied[jointIndex])

        print(f"  Articulación {jointIndex} ({joint_names[jointIndex]}): Ángulo = {np.degrees(jointAngle):.2f}°, "
              f"Torque Aplicado = {torques_applied[jointIndex]:.4f} Nm, "
              f"Velocidad = {jointVelocity:.4f} rad/s")

    # Al acabar la flexión, reseteamos el estado para la extensión
    if step == total_steps_flexion - 1:
        print("\n[MODIFICACIÓN] Reseteando el estado para la extensión...")
        # Guardamos los ángulos finales de flexión como nuevos ceros
        for jIdx in range(numLinks):
            final_flexion_angles[jIdx] = p.getJointState(finger, jIdx)[0]
        
        # Colocamos las articulaciones en esa posición con velocidad cero
        for jointIndex in range(numLinks):
            p.resetJointState(finger, jointIndex, targetValue=final_flexion_angles[jointIndex], targetVelocity=0.0)
            p.setJointMotorControl2(
                bodyIndex=finger,
                jointIndex=jointIndex,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0
            )

# In[18]:
matplotlib.use('tkagg')
plt.figure(figsize=(20, 6))
for jointIndex in range(numLinks):
    plt.plot(times, jointAngles[jointIndex], label=f'Articulación {joint_names[jointIndex]}')
plt.axvline(x=total_steps_flexion * time_step, color='k', linestyle='--', label='Inicio Extensión')
plt.xlabel('Tiempo (s)')
plt.ylabel('Ángulo (°)')
plt.title('Evolución de los Ángulos de las Articulaciones con Flexión y Extensión')
plt.legend()
plt.grid(True)
plt.show()

# In[19]:
jointAngles_np = np.array(jointAngles)
times_np = np.array(times)

flexion_indices = slice(0, total_steps_flexion)
extension_indices = slice(total_steps_flexion, total_steps_total)

data_flexion = [
    jointAngles_np[0, flexion_indices],
    jointAngles_np[1, flexion_indices],
    jointAngles_np[2, flexion_indices],
    jointAngles_np[3, flexion_indices],
    times_np[flexion_indices]
]

data_extension = [
    jointAngles_np[0, extension_indices],
    jointAngles_np[1, extension_indices],
    jointAngles_np[2, extension_indices],
    jointAngles_np[3, extension_indices],
    times_np[extension_indices]
]

data = np.array([data_flexion, data_extension])
np.save('datos_angulos_29.npy', data)
print(f"Datos guardados con forma: {data.shape}")

# In[20]:
plt.figure(figsize=(20, 6))
for jointIndex in range(numLinks):
    plt.plot(times, appliedTorques[jointIndex], label=f'Articulación {joint_names[jointIndex]}')
plt.axvline(x=total_steps_flexion * time_step, color='k', linestyle='--', label='Inicio Extensión')
plt.xlabel('Tiempo (s)')
plt.ylabel('Torque Aplicado (Nm)')
plt.title('Torques Aplicados en las Articulaciones durante Flexión y Extensión')
plt.legend()
plt.grid(True)
plt.show()

# In[21]:
p.disconnect()
# %%