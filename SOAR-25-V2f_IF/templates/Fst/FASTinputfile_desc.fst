------- OpenFAST INPUT FILE -------------------------------------------
Generated with AeroElasticSE FAST driver
---------------------- SIMULATION CONTROL --------------------------------------
                Echo        - Echo input data to <RootName>.ech (flag)
                AbortLevel  - Error level when simulation should abort (string) {"WARNING", "SEVERE", "FATAL"}
                TMax        - Total run time (s)
                DT          - Recommended module time step (s)
                InterpOrder - Interpolation order for input/output time history (-) {1=linear, 2=quadratic}
                NumCrctn    - Number of correction iterations (-) {0=explicit calculation, i.e., no corrections}
                DT_UJac     - Time between calls to get Jacobians (s)
                UJacSclFact - Scaling factor used in Jacobians (-)
---------------------- FEATURE SWITCHES AND FLAGS ------------------------------
                CompElast   - Compute structural dynamics (switch) {1=ElastoDyn; 2=ElastoDyn + BeamDyn for blades}
                CompInflow  - Compute inflow wind velocities (switch) {0=still air; 1=InflowWind; 2=external from OpenFOAM}
                CompAero    - Compute aerodynamic loads (switch) {0=None; 1=AeroDyn v14; 2=AeroDyn v15}
                CompServo   - Compute control and electrical-drive dynamics (switch) {0=None; 1=ServoDyn}
                CompHydro   - Compute hydrodynamic loads (switch) {0=None; 1=HydroDyn}
                CompSub     - Compute sub-structural dynamics (switch) {0=None; 1=SubDyn; 2=External Platform MCKF}
                CompMooring - Compute mooring system (switch) {0=None; 1=MAP++; 2=FEAMooring; 3=MoorDyn; 4=OrcaFlex}
                CompIce     - Compute ice loads (switch) {0=None; 1=IceFloe; 2=IceDyn}
                MHK         - MHK turbine type (switch) {0=Not an MHK turbine; 1=Fixed MHK turbine; 2=Floating MHK turbine}
---------------------- ENVIRONMENTAL CONDITIONS --------------------------------
                Gravity         - Gravitational acceleration (m/s^2)
                AirDens         - Air density (kg/m^3)
                WtrDens         - Water density (kg/m^3)
                KinVisc         - Kinematic viscosity of working fluid (m^2/s)
                SpdSound        - Speed of sound in working fluid (m/s)
                Patm            - Atmospheric pressure (Pa) [used only for an MHK turbine cavitation check]
                Pvap            - Vapour pressure of working fluid (Pa) [used only for an MHK turbine cavitation check]
                WtrDpth         - Water depth (m)
                MSL2SWL         - Offset between still-water level and mean sea level (m) [positive upward]
---------------------- INPUT FILES ---------------------------------------------
                EDFile      - Name of file containing ElastoDyn input parameters (quoted string)
                BDBldFile(1) - Name of file containing BeamDyn input parameters for blade 1 (quoted string)
                BDBldFile(2) - Name of file containing BeamDyn input parameters for blade 2 (quoted string)
                BDBldFile(3) - Name of file containing BeamDyn input parameters for blade 3 (quoted string)
                InflowFile  - Name of file containing inflow wind input parameters (quoted string)
                AeroFile    - Name of file containing aerodynamic input parameters (quoted string)
                ServoFile   - Name of file containing control and electrical-drive input parameters (quoted string)
                HydroFile   - Name of file containing hydrodynamic input parameters (quoted string)
                SubFile     - Name of file containing sub-structural input parameters (quoted string)
                MooringFile - Name of file containing mooring system input parameters (quoted string)
                IceFile     - Name of file containing ice input parameters (quoted string)
---------------------- OUTPUT --------------------------------------------------
                SumPrint    - Print summary data to "<RootName>.sum" (flag)
                SttsTime    - Amount of time between screen status messages (s)
                ChkptTime   - Amount of time between creating checkpoint files for potential restart (s)
                DT_Out      - Time step for tabular output (s) (or "default")
                TStart      - Time to begin tabular output (s)
                OutFileFmt  - Format for tabular (time-marching) output file (switch) {1: text file [<RootName>.out], 2: binary file [<RootName>.outb], 3: both}
                TabDelim    - Use tab delimiters in text tabular output file? (flag) {uses spaces if false}
                OutFmt      - Format used for text tabular output, excluding the time channel.  Resulting field should be 10 characters. (quoted string)
---------------------- LINEARIZATION -------------------------------------------
                Linearize   - Linearization analysis (flag)
                CalcSteady  - Calculate a steady-state periodic operating point before linearization? [unused if Linearize=False] (flag)
                TrimCase    - Controller parameter to be trimmed {1:yaw; 2:torque; 3:pitch} [used only if CalcSteady=True] (-)
                TrimTol     - Tolerance for the rotational speed convergence [used only if CalcSteady=True] (-)
                TrimGain    - Proportional gain for the rotational speed error (>0) [used only if CalcSteady=True] (rad/(rad/s) for yaw or pitch; Nm/(rad/s) for torque)
                Twr_Kdmp    - Damping factor for the tower [used only if CalcSteady=True] (N/(m/s))
                Bld_Kdmp    - Damping factor for the blades [used only if CalcSteady=True] (N/(m/s))
                NLinTimes   - Number of times to linearize (-) [>=1] [unused if Linearize=False]
                LinTimes    - List of times at which to linearize (s) [1 to NLinTimes] [used only when Linearize=True and CalcSteady=False]
                LinInputs   - Inputs included in linearization (switch) {0=none; 1=standard; 2=all module inputs (debug)} [unused if Linearize=False]
                LinOutputs  - Outputs included in linearization (switch) {0=none; 1=from OutList(s); 2=all module outputs (debug)} [unused if Linearize=False]
                LinOutJac   - Include full Jacobians in linearization output (for debug) (flag) [unused if Linearize=False; used only if LinInputs=LinOutputs=2]
                LinOutMod   - Write module-level linearization output files in addition to output for full system? (flag) [unused if Linearize=False]
---------------------- VISUALIZATION ------------------------------------------
                WrVTK       - VTK visualization data output: (switch) {0=none; 1=initialization data only; 2=animation}
                VTK_type    - Type of VTK visualization data: (switch) {1=surfaces; 2=basic meshes (lines/points); 3=all meshes (debug)} [unused if WrVTK=0]
                VTK_fields  - Write mesh fields to VTK data files? (flag) {true/false} [unused if WrVTK=0]
                VTK_fps     - Frame rate for VTK output (frames per second){will use closest integer multiple of DT} [used only if WrVTK=2]
