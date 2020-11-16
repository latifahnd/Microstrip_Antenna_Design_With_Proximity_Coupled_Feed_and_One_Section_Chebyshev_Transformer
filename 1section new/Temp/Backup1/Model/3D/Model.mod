'# MWS Version: Version 2019.1 - Oct 19 2018 - ACIS 28.0.2 -

'# length = mm
'# frequency = GHz
'# time = ns
'# frequency range: fmin = 1 fmax = 10
'# created = '[VERSION]2019.1|28.0.2|20181019[/VERSION]


'@ use template: Antenna - Planar_9.cfg

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
'set the units
With Units
    .Geometry "mm"
    .Frequency "GHz"
    .Voltage "V"
    .Resistance "Ohm"
    .Inductance "H"
    .TemperatureUnit  "Kelvin"
    .Time "ns"
    .Current "A"
    .Conductance "Siemens"
    .Capacitance "F"
End With
'----------------------------------------------------------------------------
'set the frequency range
Solver.FrequencyRange "1", "10"
'----------------------------------------------------------------------------
Plot.DrawBox True
With Background
     .Type "Normal"
     .Epsilon "1.0"
     .Mu "1.0"
     .XminSpace "0.0"
     .XmaxSpace "0.0"
     .YminSpace "0.0"
     .YmaxSpace "0.0"
     .ZminSpace "0.0"
     .ZmaxSpace "0.0"
End With
With Boundary
     .Xmin "expanded open"
     .Xmax "expanded open"
     .Ymin "expanded open"
     .Ymax "expanded open"
     .Zmin "expanded open"
     .Zmax "expanded open"
     .Xsymmetry "none"
     .Ysymmetry "none"
     .Zsymmetry "none"
End With
' optimize mesh settings for planar structures
With Mesh
     .MergeThinPECLayerFixpoints "True"
     .RatioLimit "20"
     .AutomeshRefineAtPecLines "True", "6"
     .FPBAAvoidNonRegUnite "True"
     .ConsiderSpaceForLowerMeshLimit "False"
     .MinimumStepNumber "5"
     .AnisotropicCurvatureRefinement "True"
     .AnisotropicCurvatureRefinementFSM "True"
End With
With MeshSettings
     .SetMeshType "Hex"
     .Set "RatioLimitGeometry", "20"
     .Set "EdgeRefinementOn", "1"
     .Set "EdgeRefinementRatio", "6"
End With
With MeshSettings
     .SetMeshType "HexTLM"
     .Set "RatioLimitGeometry", "20"
End With
With MeshSettings
     .SetMeshType "Tet"
     .Set "VolMeshGradation", "1.5"
     .Set "SrfMeshGradation", "1.5"
End With
' change mesh adaption scheme to energy
' 		(planar structures tend to store high energy
'     	 locally at edges rather than globally in volume)
MeshAdaption3D.SetAdaptionStrategy "Energy"
' switch on FD-TET setting for accurate farfields
FDSolver.ExtrudeOpenBC "True"
PostProcess1D.ActivateOperation "vswr", "true"
PostProcess1D.ActivateOperation "yz-matrices", "true"
With FarfieldPlot
	.ClearCuts ' lateral=phi, polar=theta
	.AddCut "lateral", "0", "1"
	.AddCut "lateral", "90", "1"
	.AddCut "polar", "90", "1"
End With
'----------------------------------------------------------------------------
Dim sDefineAt As String
sDefineAt = "1;5.5;10"
Dim sDefineAtName As String
sDefineAtName = "1;5.5;10"
Dim sDefineAtToken As String
sDefineAtToken = "f="
Dim aFreq() As String
aFreq = Split(sDefineAt, ";")
Dim aNames() As String
aNames = Split(sDefineAtName, ";")
Dim nIndex As Integer
For nIndex = LBound(aFreq) To UBound(aFreq)
Dim zz_val As String
zz_val = aFreq (nIndex)
Dim zz_name As String
zz_name = sDefineAtToken & aNames (nIndex)
' Define E-Field Monitors
With Monitor
    .Reset
    .Name "e-field ("& zz_name &")"
    .Dimension "Volume"
    .Domain "Frequency"
    .FieldType "Efield"
    .MonitorValue  zz_val
    .Create
End With
' Define H-Field Monitors
With Monitor
    .Reset
    .Name "h-field ("& zz_name &")"
    .Dimension "Volume"
    .Domain "Frequency"
    .FieldType "Hfield"
    .MonitorValue  zz_val
    .Create
End With
Next
'----------------------------------------------------------------------------
With MeshSettings
     .SetMeshType "Hex"
     .Set "Version", 1%
End With
With Mesh
     .MeshType "PBA"
End With
'set the solver type
ChangeSolverType("HF Time Domain")
'----------------------------------------------------------------------------

'@ define material: Copper (annealed)

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Material
     .Reset
     .Name "Copper (annealed)"
     .Folder ""
     .FrqType "static"
     .Type "Normal"
     .SetMaterialUnit "Hz", "mm"
     .Epsilon "1"
     .Mu "1.0"
     .Kappa "5.8e+007"
     .TanD "0.0"
     .TanDFreq "0.0"
     .TanDGiven "False"
     .TanDModel "ConstTanD"
     .KappaM "0"
     .TanDM "0.0"
     .TanDMFreq "0.0"
     .TanDMGiven "False"
     .TanDMModel "ConstTanD"
     .DispModelEps "None"
     .DispModelMu "None"
     .DispersiveFittingSchemeEps "Nth Order"
     .DispersiveFittingSchemeMu "Nth Order"
     .UseGeneralDispersionEps "False"
     .UseGeneralDispersionMu "False"
     .FrqType "all"
     .Type "Lossy metal"
     .SetMaterialUnit "GHz", "mm"
     .Mu "1.0"
     .Kappa "5.8e+007"
     .Rho "8930.0"
     .ThermalType "Normal"
     .ThermalConductivity "401.0"
     .HeatCapacity "0.39"
     .MetabolicRate "0"
     .BloodFlow "0"
     .VoxelConvection "0"
     .MechanicsType "Isotropic"
     .YoungsModulus "120"
     .PoissonsRatio "0.33"
     .ThermalExpansionRate "17"
     .Colour "1", "1", "0"
     .Wireframe "False"
     .Reflection "False"
     .Allowoutline "True"
     .Transparentoutline "False"
     .Transparency "0"
     .Create
End With

'@ new component: component1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Component.New "component1"

'@ define brick: component1:Groundplane

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Brick
     .Reset 
     .Name "Groundplane" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .Xrange "-Wg/2", "Wg/2" 
     .Yrange "-Lg/2", "Lg/x" 
     .Zrange "0", "t" 
     .Create
End With

'@ define material: Rogers RT5880 (lossy)

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Material
     .Reset
     .Name "Rogers RT5880 (lossy)"
     .Folder ""
     .FrqType "all"
     .Type "Normal"
     .SetMaterialUnit "GHz", "mm"
     .Epsilon "2.2"
     .Mu "1.0"
     .Kappa "0.0"
     .TanD "0.0009"
     .TanDFreq "10.0"
     .TanDGiven "True"
     .TanDModel "ConstTanD"
     .KappaM "0.0"
     .TanDM "0.0"
     .TanDMFreq "0.0"
     .TanDMGiven "False"
     .TanDMModel "ConstKappa"
     .DispModelEps "None"
     .DispModelMu "None"
     .DispersiveFittingSchemeEps "General 1st"
     .DispersiveFittingSchemeMu "General 1st"
     .UseGeneralDispersionEps "False"
     .UseGeneralDispersionMu "False"
     .Rho "0.0"
     .ThermalType "Normal"
     .ThermalConductivity "0.20"
     .SetActiveMaterial "all"
     .Colour "0.94", "0.82", "0.76"
     .Wireframe "False"
     .Transparency "0"
     .Create
End With

'@ define brick: component1:Substrat-2

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Brick
     .Reset 
     .Name "Substrat-2" 
     .Component "component1" 
     .Material "Rogers RT5880 (lossy)" 
     .Xrange "-Wg/2", "Wg/2" 
     .Yrange "-Lg/2", "Lg/x" 
     .Zrange "t", "t+h" 
     .Create
End With

'@ define brick: component1:Feed-0

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Brick
     .Reset 
     .Name "Feed-0" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .Xrange "-Wf0/2", "Wf0/2" 
     .Yrange "-Lg/2", "-Lg/2+Lf0" 
     .Zrange "t+h", "t+h+t" 
     .Create
End With

'@ define brick: component1:Feed-1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Brick
     .Reset 
     .Name "Feed-1" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .Xrange "-Wf1/2-d1/2", "Wf1/2+d1/2" 
     .Yrange "-Lg/2+Lf0", "-Lg/2+Lf0+Lf" 
     .Zrange "t+h", "t+h+t" 
     .Create
End With

'@ boolean add shapes: component1:Feed-0, component1:Feed-1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Solid.Add "component1:Feed-0", "component1:Feed-1"

'@ define brick: component1:Substrat-1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Brick
     .Reset 
     .Name "Substrat-1" 
     .Component "component1" 
     .Material "Rogers RT5880 (lossy)" 
     .Xrange "-Wg/2", "Wg/2" 
     .Yrange "-Lg/2", "Lg/x" 
     .Zrange "t+h+t", "t+h+t+h" 
     .Create
End With

'@ define brick: component1:Patch

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Brick
     .Reset 
     .Name "Patch" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .Xrange "-Wp/2", "Wp/2" 
     .Yrange "-Lp/2", "Lp/2" 
     .Zrange "t+h+t+h", "t+h+t+h+t" 
     .Create
End With

'@ pick face

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Pick.PickFaceFromId "component1:Feed-0", "9"

'@ define port:1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
' Port constructed by macro Solver -> Ports -> Calculate port extension coefficient
With Port
  .Reset
  .PortNumber "1"
  .NumberOfModes "1"
  .AdjustPolarization False
  .PolarizationAngle "0.0"
  .ReferencePlaneDistance "0"
  .TextSize "50"
  .Coordinates "Picks"
  .Orientation "Positive"
  .PortOnBound "True"
  .ClipPickedPortToBound "False"
  .XrangeAdd "1.5875*1.26", "1.5875*1.26"
  .YrangeAdd "0", "0"
  .ZrangeAdd "1.57", "1.57"
  .Shield "PEC"
  .SingleEnded "False"
  .Create
End With

'@ define special time domain solver parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
'STEADY STATE
With Solver
     .SteadyStateDurationType "Number of pulses"
     .NumberOfPulseWidths "5"
     .SteadyStateDurationTime "3.88159"
     .SteadyStateDurationTimeAsDistance "1164.48"
     .StopCriteriaShowExcitation "False"
     .RemoveAllStopCriteria
     .AddStopCriterion "All S-Parameters", "0.004", "1", "False"
     .AddStopCriterion "Transmission S-Parameters", "0.004", "1", "False"
     .AddStopCriterion "Reflection S-Parameters", "0.004", "1", "False"
     .AddStopCriterion "All Probes", "0.004", "1", "False"
     .AddStopCriterion "All Radiated Powers", "0.004", "1", "False"
End With
'GENERAL
With Solver
     .TimeStepStabilityFactor "1.0"
     .RestartAfterInstabilityAbort "True"
     .AutomaticTimeSignalSampling "True"
     .SuppressTimeSignalStorage "False"
     .ConsiderExcitationForFreqSamplingRate "False"
     .UseBroadBandPhaseShift "False"
     .SetBroadBandPhaseShiftLowerBoundFac "0.1"
     .SetPortShieldingType "NONE"
     .FrequencySamples "901"
     .FrequencyLogSamples "0"
     .ConsiderTwoPortReciprocity "True"
     .EnergyBalanceLimit "0.03"
     .TDRComputation "False"
     .TDRShift50Percent "False"
     .AutoDetectIdenticalPorts "False"
End With
'HEXAHEDRAL
With Solver
     .SetPMLType "CONVPML"
     .UseVariablePMLLayerSizeStandard "False"
     .KeepPMLDepthDuringMeshAdaptationWithVariablePMLLayerSize "False"
     .SetSubcycleState "Automatic"
     .NormalizeToReferenceSignal "False"
     .SetEnhancedPMLStabilization "Automatic"
     .SimplifiedPBAMethod "False"
     .SParaAdjustment "True"
     .PrepareFarfields "True"
     .MonitorFarFieldsNearToModel "True"
     .DiscreteItemUpdate "Distributed"
End With
'MATERIAL
With Solver
     .SurfaceImpedanceOrder "10"
     .ActivatePowerLoss1DMonitor "True"
     .PowerLoss1DMonitorPerSolid "False"
     .Use3DFieldMonitorForPowerLoss1DMonitor "True"
     .UseFarFieldMonitorForPowerLoss1DMonitor "False"
     .UseExtraFreqForPowerLoss1DMonitor "False"
     .ResetPowerLoss1DMonitorExtraFreq
     .SetDispNonLinearMaterialMonitor "False"
     .ActivateDispNonLinearMaterialMonitor "0.0",  "0.02",  "0.0",  "False"
     .SetTimePowerLossSIMaterialMonitor "False"
     .ActivateTimePowerLossSIMaterialMonitor "0.0",  "0.02",  "0.0",  "False"
     .SetTimePowerLossSIMaterialMonitorAverage "False"
     .SetTimePowerLossSIMaterialMonitorAverageRepPeriod "0.0"
     .TimePowerLossSIMaterialMonitorPerSolid "False"
     .ActivateSpaceMaterial3DMonitor "False"
     .Use3DFieldMonitorForSpaceMaterial3DMonitor "True"
     .UseExtraFreqForSpaceMaterial3DMonitor "False"
     .ResetSpaceMaterial3DMonitorExtraFreq
     .SetHFTDDispUpdateScheme "Standard"
End With
'AR-FILTER
With Solver
     .UseArfilter "False"
     .ArMaxEnergyDeviation "0.1"
     .ArPulseSkip "1"
End With
'WAVEGUIDE
With Solver
     .WaveguidePortGeneralized "True"
     .WaveguidePortModeTracking "False"
     .WaveguidePortROM "False"
     .DispEpsFullDeembedding "False"
     .SetSamplesFullDeembedding "20"
     .AbsorbUnconsideredModeFields "Automatic"
     .SetModeFreqFactor "0.5"
     .AdaptivePortMeshing "True"
     .AccuracyAdaptivePortMeshing "1"
     .PassesAdaptivePortMeshing "4"
End With
'HEXAHEDRAL TLM
With Solver
     .AnisotropicSheetSurfaceType "0"
     .MultiStrandedCableRoute "False"
     .UseAbsorbingBoundary "True"
     .UseDoublePrecision "False"
     .AllowMaterialOverlap "True"
     .ExcitePlanewaveNearModel "False"
     .SetGroundPlane "False"
     .GroundPlane "x", "0.0"
     .NumberOfLayers "5"
     .AverageFieldProbe "False"
     .NormalizeToGaussian "True"
     .TimeSignalSamplingFactor "1"
End With
'TLM POSTPROCESSING
With Solver
     .ResetSettings
     .CalculateNearFieldOnCylindricalSurfaces "false", "Coarse" 
     .CylinderGridCustomStep "1" 
     .CalculateNearFieldOnCircularCuts "false" 
     .CylinderBaseCenter "0", "0", "0" 
     .CylinderRadius "3" 
     .CylinderHeight "3" 
     .CylinderSpacing "1" 
     .CylinderResolution "2.0" 
     .CylinderAllPolarization "true" 
     .CylinderRadialAngularVerticalComponents "false" 
     .CylinderMagnitudeOfTangentialConponent "false" 
     .CylinderVm "true" 
     .CylinderDBVm "false" 
     .CylinderDBUVm "false" 
     .CylinderAndFrontAxes "+y", "+z" 
     .ApplyLinearPrediction "false" 
     .Windowing "None" 
     .LogScaleFrequency "false" 
     .AutoFreqStep "true", "1"
     .SetExcitationSignal "" 
     .SaveSettings
End With

'@ define time domain solver parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Mesh.SetCreator "High Frequency" 
With Solver 
     .Method "Hexahedral"
     .CalculationType "TD-S"
     .StimulationPort "All"
     .StimulationMode "All"
     .SteadyStateLimit "-40"
     .MeshAdaption "False"
     .AutoNormImpedance "True"
     .NormingImpedance "50"
     .CalculateModesOnly "False"
     .SParaSymmetry "False"
     .StoreTDResultsInCache  "False"
     .FullDeembedding "False"
     .SuperimposePLWExcitation "False"
     .UseSensitivityAnalysis "False"
End With

'@ set PBA version

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Discretizer.PBAVersion "2018101919"

'@ set optimizer parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer
  .SetMinMaxAuto "30" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d1", "False" 
  .SetParameterInit "0" 
  .SetParameterMin "-0.1" 
  .SetParameterMax "0.1" 
  .SetParameterAnchors "5" 
  .SelectParameter "h", "False" 
  .SetParameterInit "1.57" 
  .SetParameterMin "1.413" 
  .SetParameterMax "1.727" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf", "False" 
  .SetParameterInit "9.96" 
  .SetParameterMin "8.964" 
  .SetParameterMax "10.956" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf0", "False" 
  .SetParameterInit "5" 
  .SetParameterMin "4.5" 
  .SetParameterMax "5.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lg", "False" 
  .SetParameterInit "29.920000000000002" 
  .SetParameterMin "24.183" 
  .SetParameterMax "29.557" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lp", "True" 
  .SetParameterInit "17.45" 
  .SetParameterMin "12.215" 
  .SetParameterMax "22.685" 
  .SetParameterAnchors "5" 
  .SelectParameter "t", "False" 
  .SetParameterInit "0.035" 
  .SetParameterMin "0.0315" 
  .SetParameterMax "0.0385" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf0", "False" 
  .SetParameterInit "4.88" 
  .SetParameterMin "4.392" 
  .SetParameterMax "5.368" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf1", "False" 
  .SetParameterInit "0.63" 
  .SetParameterMin "0.567" 
  .SetParameterMax "0.693" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wg", "False" 
  .SetParameterInit "30.97" 
  .SetParameterMin "27.873" 
  .SetParameterMax "34.067" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wp", "True" 
  .SetParameterInit "21.55" 
  .SetParameterMin "15.085" 
  .SetParameterMax "28.015" 
  .SetParameterAnchors "5" 
  .SelectParameter "x", "False" 
  .SetParameterInit "2" 
  .SetParameterMin "1.8" 
  .SetParameterMax "2.2" 
  .SetParameterAnchors "5" 
End With

'@ set optimizer settings

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .SetOptimizerType "Trust_Region" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0.0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0.0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0.0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "Automatic" 
  .SetOptionMoveMesh "False" 
End With

'@ add optimizer goals: 1DC Primary Result / 0

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .AddGoal "1DC Primary Result" 
  .SetGoalOperator "<" 
  .SetGoalTarget "-14" 
  .UseSlope "False" 
  .SetGoalTargetMax "0.0" 
  .SetGoalWeight "1.0" 
  .SetGoalNormNew "SumDiff" 
  .SetGoal1DCResultName "1D Results\S-Parameters\S1,1" 
  .SetGoalScalarType "MagdB20" 
  .SetGoalRange "1", "10" 
  .SetGoalRangeType "total" 
End With

'@ add optimizer goals: 1DC Primary Result / 1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .AddGoal "1DC Primary Result" 
  .SetGoalOperator "<" 
  .SetGoalTarget "-14" 
  .UseSlope "False" 
  .SetGoalTargetMax "0.0" 
  .SetGoalWeight "1.0" 
  .SetGoalNormNew "MaxDiff" 
  .SetGoal1DCResultName "1D Results\S-Parameters\S1,1" 
  .SetGoalScalarType "MagdB20" 
  .SetGoalRange "1", "10" 
  .SetGoalRangeType "total" 
End With

'@ set optimizer parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer
  .SetMinMaxAuto "30" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d1", "False" 
  .SetParameterInit "0" 
  .SetParameterMin "-0.1" 
  .SetParameterMax "0.1" 
  .SetParameterAnchors "5" 
  .SelectParameter "h", "False" 
  .SetParameterInit "1.57" 
  .SetParameterMin "1.413" 
  .SetParameterMax "1.727" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf", "False" 
  .SetParameterInit "9.96" 
  .SetParameterMin "8.964" 
  .SetParameterMax "10.956" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf0", "False" 
  .SetParameterInit "5" 
  .SetParameterMin "4.5" 
  .SetParameterMax "5.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lg", "False" 
  .SetParameterInit "29.920000000000002" 
  .SetParameterMin "24.183" 
  .SetParameterMax "29.557" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lp", "True" 
  .SetParameterInit "17.45" 
  .SetParameterMin "12.215" 
  .SetParameterMax "22.685" 
  .SetParameterAnchors "5" 
  .SelectParameter "t", "False" 
  .SetParameterInit "0.035" 
  .SetParameterMin "0.0315" 
  .SetParameterMax "0.0385" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf0", "False" 
  .SetParameterInit "4.88" 
  .SetParameterMin "4.392" 
  .SetParameterMax "5.368" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf1", "False" 
  .SetParameterInit "0.63" 
  .SetParameterMin "0.567" 
  .SetParameterMax "0.693" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wg", "False" 
  .SetParameterInit "30.97" 
  .SetParameterMin "27.873" 
  .SetParameterMax "34.067" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wp", "True" 
  .SetParameterInit "21.55" 
  .SetParameterMin "15.085" 
  .SetParameterMax "28.015" 
  .SetParameterAnchors "5" 
  .SelectParameter "x", "False" 
  .SetParameterInit "2" 
  .SetParameterMin "1.8" 
  .SetParameterMax "2.2" 
  .SetParameterAnchors "5" 
End With

'@ set optimizer settings

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .SetOptimizerType "Trust_Region" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0.0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0.0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0.0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "Automatic" 
  .SetOptionMoveMesh "False" 
End With

'@ delete port: port1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Port.Delete "1"

'@ pick face

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Pick.PickFaceFromId "component1:Feed-0", "9"

'@ define port:1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
' Port constructed by macro Solver -> Ports -> Calculate port extension coefficient
With Port
  .Reset
  .PortNumber "1"
  .NumberOfModes "1"
  .AdjustPolarization False
  .PolarizationAngle "0.0"
  .ReferencePlaneDistance "0"
  .TextSize "50"
  .Coordinates "Picks"
  .Orientation "Positive"
  .PortOnBound "True"
  .ClipPickedPortToBound "False"
  .XrangeAdd "1.5875*1.26", "1.5875*1.26"
  .YrangeAdd "0", "0"
  .ZrangeAdd "1.57", "1.57"
  .Shield "PEC"
  .SingleEnded "False"
  .Create
End With

'@ set optimizer parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer
  .SetMinMaxAuto "25" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d1", "False" 
  .SetParameterInit "0" 
  .SetParameterMin "-0.1" 
  .SetParameterMax "0.1" 
  .SetParameterAnchors "5" 
  .SelectParameter "h", "False" 
  .SetParameterInit "1.57" 
  .SetParameterMin "1.413" 
  .SetParameterMax "1.727" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf", "False" 
  .SetParameterInit "9.55" 
  .SetParameterMin "8.964" 
  .SetParameterMax "10.956" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf0", "False" 
  .SetParameterInit "5" 
  .SetParameterMin "4.5" 
  .SetParameterMax "5.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lg", "False" 
  .SetParameterInit "31.100000000000001" 
  .SetParameterMin "24.183" 
  .SetParameterMax "29.557" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lp", "True" 
  .SetParameterInit "17.45" 
  .SetParameterMin "13.087" 
  .SetParameterMax "21.813" 
  .SetParameterAnchors "5" 
  .SelectParameter "t", "False" 
  .SetParameterInit "0.035" 
  .SetParameterMin "0.0315" 
  .SetParameterMax "0.0385" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf0", "False" 
  .SetParameterInit "4.88" 
  .SetParameterMin "4.392" 
  .SetParameterMax "5.368" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf1", "False" 
  .SetParameterInit "0.63" 
  .SetParameterMin "0.567" 
  .SetParameterMax "0.693" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wg", "False" 
  .SetParameterInit "30.97" 
  .SetParameterMin "27.873" 
  .SetParameterMax "34.067" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wp", "True" 
  .SetParameterInit "21.55" 
  .SetParameterMin "16.163" 
  .SetParameterMax "26.938" 
  .SetParameterAnchors "5" 
  .SelectParameter "x", "False" 
  .SetParameterInit "2" 
  .SetParameterMin "1.8" 
  .SetParameterMax "2.2" 
  .SetParameterAnchors "5" 
End With

'@ set optimizer settings

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .SetOptimizerType "Trust_Region" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "Automatic" 
  .SetOptionMoveMesh "False" 
End With

'@ set optimizer parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer
  .SetMinMaxAuto "25" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d1", "False" 
  .SetParameterInit "0" 
  .SetParameterMin "-0.1" 
  .SetParameterMax "0.1" 
  .SetParameterAnchors "5" 
  .SelectParameter "h", "False" 
  .SetParameterInit "1.57" 
  .SetParameterMin "1.413" 
  .SetParameterMax "1.727" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf", "False" 
  .SetParameterInit "9.55" 
  .SetParameterMin "8.964" 
  .SetParameterMax "10.956" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf0", "False" 
  .SetParameterInit "5" 
  .SetParameterMin "4.5" 
  .SetParameterMax "5.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lg", "False" 
  .SetParameterInit "31.100000000000001" 
  .SetParameterMin "24.183" 
  .SetParameterMax "29.557" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lp", "True" 
  .SetParameterInit "17.45" 
  .SetParameterMin "13.087" 
  .SetParameterMax "21.813" 
  .SetParameterAnchors "5" 
  .SelectParameter "t", "False" 
  .SetParameterInit "0.035" 
  .SetParameterMin "0.0315" 
  .SetParameterMax "0.0385" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf0", "False" 
  .SetParameterInit "4.88" 
  .SetParameterMin "4.392" 
  .SetParameterMax "5.368" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf1", "False" 
  .SetParameterInit "0.63" 
  .SetParameterMin "0.567" 
  .SetParameterMax "0.693" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wg", "False" 
  .SetParameterInit "30.97" 
  .SetParameterMin "27.873" 
  .SetParameterMax "34.067" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wp", "True" 
  .SetParameterInit "21.55" 
  .SetParameterMin "16.163" 
  .SetParameterMax "26.938" 
  .SetParameterAnchors "5" 
  .SelectParameter "x", "False" 
  .SetParameterInit "2" 
  .SetParameterMin "1.8" 
  .SetParameterMax "2.2" 
  .SetParameterAnchors "5" 
End With

'@ set optimizer settings

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .SetOptimizerType "Trust_Region" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "Automatic" 
  .SetOptionMoveMesh "False" 
End With 

'@ delete port: port1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Port.Delete "1" 


'@ pick face

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Pick.PickFaceFromId "component1:Feed-0", "9" 


'@ define port:1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
' Port constructed by macro Solver -> Ports -> Calculate port extension coefficient


With Port
  .Reset
  .PortNumber "1"
  .NumberOfModes "1"
  .AdjustPolarization False
  .PolarizationAngle "0.0"
  .ReferencePlaneDistance "0"
  .TextSize "50"
  .Coordinates "Picks"
  .Orientation "Positive"
  .PortOnBound "True"
  .ClipPickedPortToBound "False"
  .XrangeAdd "1.5875*1.26", "1.5875*1.26"
  .YrangeAdd "0", "0"
  .ZrangeAdd "1.57", "1.57"
  .Shield "PEC"
  .SingleEnded "False"
  .Create
End With



'@ set optimizer parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer
  .SetMinMaxAuto "25" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d1", "False" 
  .SetParameterInit "0" 
  .SetParameterMin "-0.1" 
  .SetParameterMax "0.1" 
  .SetParameterAnchors "5" 
  .SelectParameter "h", "False" 
  .SetParameterInit "1.57" 
  .SetParameterMin "1.413" 
  .SetParameterMax "1.727" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf", "False" 
  .SetParameterInit "9.5500000000000007" 
  .SetParameterMin "8.964" 
  .SetParameterMax "10.956" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf0", "False" 
  .SetParameterInit "4.7" 
  .SetParameterMin "4.5" 
  .SetParameterMax "5.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lg", "False" 
  .SetParameterInit "31.5" 
  .SetParameterMin "24.183" 
  .SetParameterMax "29.557" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lp", "True" 
  .SetParameterInit "17.449999999999999" 
  .SetParameterMin "13.087" 
  .SetParameterMax "21.813" 
  .SetParameterAnchors "5" 
  .SelectParameter "t", "False" 
  .SetParameterInit "0.035" 
  .SetParameterMin "0.0315" 
  .SetParameterMax "0.0385" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf0", "False" 
  .SetParameterInit "4.8799999999999999" 
  .SetParameterMin "4.392" 
  .SetParameterMax "5.368" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf1", "False" 
  .SetParameterInit "0.63" 
  .SetParameterMin "0.567" 
  .SetParameterMax "0.693" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wg", "False" 
  .SetParameterInit "30.969999999999999" 
  .SetParameterMin "27.873" 
  .SetParameterMax "34.067" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wp", "True" 
  .SetParameterInit "21.550000000000001" 
  .SetParameterMin "16.163" 
  .SetParameterMax "26.938" 
  .SetParameterAnchors "5" 
  .SelectParameter "x", "False" 
  .SetParameterInit "2" 
  .SetParameterMin "1.8" 
  .SetParameterMax "2.2" 
  .SetParameterAnchors "5" 
End With 


'@ set optimizer settings

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .SetOptimizerType "Nelder_Mead_Simplex" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "Automatic" 
  .SetOptionMoveMesh "False" 
End With 


'@ set optimizer parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer
  .SetMinMaxAuto "25" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d1", "False" 
  .SetParameterInit "0" 
  .SetParameterMin "-0.1" 
  .SetParameterMax "0.1" 
  .SetParameterAnchors "5" 
  .SelectParameter "h", "False" 
  .SetParameterInit "1.57" 
  .SetParameterMin "1.413" 
  .SetParameterMax "1.727" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf", "False" 
  .SetParameterInit "9.5500000000000007" 
  .SetParameterMin "8.964" 
  .SetParameterMax "10.956" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf0", "False" 
  .SetParameterInit "4.7" 
  .SetParameterMin "4.5" 
  .SetParameterMax "5.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lg", "False" 
  .SetParameterInit "31.5" 
  .SetParameterMin "24.183" 
  .SetParameterMax "29.557" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lp", "True" 
  .SetParameterInit "17.449999999999999" 
  .SetParameterMin "13.087" 
  .SetParameterMax "21.813" 
  .SetParameterAnchors "5" 
  .SelectParameter "t", "False" 
  .SetParameterInit "0.035" 
  .SetParameterMin "0.0315" 
  .SetParameterMax "0.0385" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf0", "False" 
  .SetParameterInit "4.8799999999999999" 
  .SetParameterMin "4.392" 
  .SetParameterMax "5.368" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf1", "False" 
  .SetParameterInit "0.63" 
  .SetParameterMin "0.567" 
  .SetParameterMax "0.693" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wg", "False" 
  .SetParameterInit "30.969999999999999" 
  .SetParameterMin "27.873" 
  .SetParameterMax "34.067" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wp", "True" 
  .SetParameterInit "21.550000000000001" 
  .SetParameterMin "16.163" 
  .SetParameterMax "26.938" 
  .SetParameterAnchors "5" 
  .SelectParameter "x", "False" 
  .SetParameterInit "2" 
  .SetParameterMin "1.8" 
  .SetParameterMax "2.2" 
  .SetParameterAnchors "5" 
End With 


'@ set optimizer settings

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .SetOptimizerType "Nelder_Mead_Simplex" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "Automatic" 
  .SetOptionMoveMesh "False" 
End With 


'@ set optimizer parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer
  .SetMinMaxAuto "2" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d1", "True" 
  .SetParameterInit "2.9" 
  .SetParameterMin "2.842" 
  .SetParameterMax "2.958" 
  .SetParameterAnchors "5" 
  .SelectParameter "h", "False" 
  .SetParameterInit "1.57" 
  .SetParameterMin "1.413" 
  .SetParameterMax "1.727" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf", "False" 
  .SetParameterInit "9.5500000000000007" 
  .SetParameterMin "8.964" 
  .SetParameterMax "10.956" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf0", "False" 
  .SetParameterInit "4.7" 
  .SetParameterMin "4.5" 
  .SetParameterMax "5.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lg", "False" 
  .SetParameterInit "31.5" 
  .SetParameterMin "24.183" 
  .SetParameterMax "29.557" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lp", "False" 
  .SetParameterInit "15.54" 
  .SetParameterMin "13.087" 
  .SetParameterMax "21.813" 
  .SetParameterAnchors "5" 
  .SelectParameter "t", "False" 
  .SetParameterInit "0.035" 
  .SetParameterMin "0.0315" 
  .SetParameterMax "0.0385" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf0", "False" 
  .SetParameterInit "4.8799999999999999" 
  .SetParameterMin "4.392" 
  .SetParameterMax "5.368" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf1", "False" 
  .SetParameterInit "0.63" 
  .SetParameterMin "0.567" 
  .SetParameterMax "0.693" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wg", "False" 
  .SetParameterInit "30.969999999999999" 
  .SetParameterMin "27.873" 
  .SetParameterMax "34.067" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wp", "False" 
  .SetParameterInit "23.08" 
  .SetParameterMin "16.163" 
  .SetParameterMax "26.938" 
  .SetParameterAnchors "5" 
  .SelectParameter "x", "False" 
  .SetParameterInit "2" 
  .SetParameterMin "1.8" 
  .SetParameterMax "2.2" 
  .SetParameterAnchors "5" 
End With 


'@ set optimizer settings

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .SetOptimizerType "Nelder_Mead_Simplex" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "Automatic" 
  .SetOptionMoveMesh "False" 
End With 


'@ delete optimizer goal: 1DC Primary Result / 1

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
Optimizer.DeleteGoal "1" 


'@ set optimizer parameters

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer
  .SetMinMaxAuto "2" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d1", "True" 
  .SetParameterInit "2.9" 
  .SetParameterMin "2.842" 
  .SetParameterMax "2.958" 
  .SetParameterAnchors "5" 
  .SelectParameter "h", "False" 
  .SetParameterInit "1.57" 
  .SetParameterMin "1.413" 
  .SetParameterMax "1.727" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf", "False" 
  .SetParameterInit "9.5500000000000007" 
  .SetParameterMin "8.964" 
  .SetParameterMax "10.956" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lf0", "False" 
  .SetParameterInit "4.7" 
  .SetParameterMin "4.5" 
  .SetParameterMax "5.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lg", "False" 
  .SetParameterInit "31.5" 
  .SetParameterMin "24.183" 
  .SetParameterMax "29.557" 
  .SetParameterAnchors "5" 
  .SelectParameter "Lp", "False" 
  .SetParameterInit "15.54" 
  .SetParameterMin "13.087" 
  .SetParameterMax "21.813" 
  .SetParameterAnchors "5" 
  .SelectParameter "t", "False" 
  .SetParameterInit "0.035" 
  .SetParameterMin "0.0315" 
  .SetParameterMax "0.0385" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf0", "False" 
  .SetParameterInit "4.8799999999999999" 
  .SetParameterMin "4.392" 
  .SetParameterMax "5.368" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wf1", "False" 
  .SetParameterInit "0.63" 
  .SetParameterMin "0.567" 
  .SetParameterMax "0.693" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wg", "False" 
  .SetParameterInit "30.969999999999999" 
  .SetParameterMin "27.873" 
  .SetParameterMax "34.067" 
  .SetParameterAnchors "5" 
  .SelectParameter "Wp", "False" 
  .SetParameterInit "23.08" 
  .SetParameterMin "16.163" 
  .SetParameterMax "26.938" 
  .SetParameterAnchors "5" 
  .SelectParameter "x", "False" 
  .SetParameterInit "2" 
  .SetParameterMin "1.8" 
  .SetParameterMax "2.2" 
  .SetParameterAnchors "5" 
End With 


'@ set optimizer settings

'[VERSION]2019.1|28.0.2|20181019[/VERSION]
With Optimizer 
  .SetOptimizerType "Nelder_Mead_Simplex" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "Automatic" 
  .SetOptionMoveMesh "False" 
End With 


