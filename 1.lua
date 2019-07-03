function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)
    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d=sim.getObjectHandle('Quadricopter_base')
    hand_handle=sim.getObjectHandle('JacoHand')
    quadricopter=sim.getObjectHandle('Quadricopter')
    quadricopter_prop_respondable1=sim.getObjectHandle('Quadricopter_propeller_respondable1')

    particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
    simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(simulateParticles))

    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    heli=sim.getObjectAssociatedWithScript(sim.handle_self)
    hand_script_handle = sim.getScriptHandle('JacoHand')
    print('hand_script_handle', hand_script_handle)

    particlesTargetVelocities={0,0,0,0}

    pParam=6
    iParam=0.04
    dParam=0.08
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    alphaCumul=0
    betaCumul=0
    rotCorrCumul=0
    psp2=0
    psp1=0
    spCumul=0

    prevEuler=0


    fakeShadow=sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end

    -- Prepare 2 floating views with the zed camera views:
    zed_vision0 = sim.getObjectHandle('zed_vision0')
    zed_vision1 = sim.getObjectHandle('zed_vision1')

    zed_v0_View=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    zed_v1_View=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(zed_v0_View,zed_vision0,64)
    sim.adjustView(zed_v1_View,zed_vision1,64)

    end_vector = {0,0,0.14}
    t_sim_start = sim.getSimulationTime()
    grapped = false
    speed = -1 -- m/s
    hold_time = 0.5 -- s
    distance_hold = 0.11
    start_position = sim.getObjectPosition(targetObj, -1)
    ----- the commented part is the decision logic to grap a 'Sphere'
    --hold_target_handle = sim.getObjectHandle('Sphere')
    --hold_target_position = sim.getObjectPosition(hold_target_handle, -1)
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(zed_v0_View)
    sim.floatingViewRemove(zed_v1_View)
    sim.clearStringSignal("rotorPower")
end 

function sysCall_actuation() 
    s=sim.getObjectSizeFactor(d)
    pos=sim.getObjectPosition(d,-1)
    target_pos = sim.getObjectPosition(targetObj, -1)
--    z_distance = target_pos[3] - hold_target_position[3]
--    print('z_distance', z_distance)
--    if (math.abs(z_distance) <  0.21) then
--        sim.setScriptSimulationParameter(hand_script_handle, 'close_hand', 'true')
--        print('Closing hand')
--    end
--    print('simulation time', sim.getSimulationTime())

    pos_z_delta = 0
--    if grapped == false then
--        if (z_distance > distance_hold) then
--                pos_z_delta = speed * sim.getSimulationTimeStep() 
--                hold_start_time = sim.getSimulationTime()
--                print('start', pos_z_delta)
--        elseif z_distance < distance_hold then
            -- hold for a while
--            if (sim.getSimulationTime() - hold_start_time) > hold_time then 
--                grapped = true
--                speed = 1
--            end
--        end
--    else
--        end_delta = start_position[3] - target_pos[3]
--        if (end_delta > 0.01) then
--            pos_z_delta = speed * sim.getSimulationTimeStep() 
--        end
--    end
    
    sim.setObjectPosition(targetObj, -1, {target_pos[1], target_pos[2], target_pos[3] + pos_z_delta})
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end
    ------------------ Controller -------------------------------------
    
    -- Send the desired motor velocities to the 4 rotors:
    data=sim.getStringSignal("rotorPower")

    -- which external forcing functions to use
    -- 0 will use no forcing functions
    --forcefunc=sim.getIntegerSignal("ForceFunction")
    power = 9
    if (data ~= nil) then
        vector=sim.unpackFloatTable(data)
        if (vector[1] ~= nil) then
            power = vector[1]
        end
    end
    print('power',power)
    

    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj,-1)
    pos=sim.getObjectPosition(d,-1)
    l=sim.getVelocity(heli)
    e_z=(targetPos[3]-pos[3])
    cumul=cumul+e_z
    thrust=power+pParam*e_z+iParam*cumul+dParam*(e_z-lastE)+l[3]*vParam
    lastE=e_z
    
    -- Horizontal control: 
    sp=sim.getObjectPosition(targetObj,d)
    m=sim.getObjectMatrix(d,-1)
    -- print(m)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    -- print('vx',vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCumul=alphaCumul + alphaE
    alphaCorr=0.3*alphaE+1.1*(alphaE-pAlphaE)-- + alphaCumul*0.001
    betaE=(vx[3]-m[12])
    betaCumul=betaCumul + betaE
    betaCorr=-0.6*betaE-1.1*(betaE-pBetaE) - betaCumul*0.001
    pAlphaE=alphaE
    pBetaE=betaE
    spCumul=spCumul + sp[2]
    alphaCorr=alphaCorr+sp[2]*0.15+4*(sp[2]-psp2) + 0.005 * spCumul
    betaCorr=betaCorr-sp[1]*0.08-2*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    -- Rotational control:
    euler=sim.getObjectOrientation(d,targetObj)
    rotCorrCumul = rotCorrCumul + euler[3]
    rotCorr=euler[3]*4 + 1*(euler[3]-prevEuler) + 0.001 * rotCorrCumul
    prevEuler=euler[3]
    
    -- Decide of the motor velocities:
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)


    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end
end 
