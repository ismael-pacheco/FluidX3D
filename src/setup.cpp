#include "setup.hpp"



#ifdef BENCHMARK
#include "info.hpp"
/*void main_setup() { // benchmark; required extensions in defines.hpp: BENCHMARK, optionally FP16S or FP16C
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	uint mlups = 0u; {

		//LBM lbm( 32u,  32u,  32u, 1.0f);
		//LBM lbm( 64u,  64u,  64u, 1.0f);
		//LBM lbm(128u, 128u, 128u, 1.0f);
		LBM lbm(256u, 256u, 256u, 1.0f); // default
		//LBM lbm(384u, 384u, 384u, 1.0f);
		//LBM lbm(512u, 512u, 512u, 1.0f);

		//const uint memory = 1488u; // memory occupation in MB (for multi-GPU benchmarks: make this close to as large as the GPU's VRAM capacity)
		//const uint3 lbm_N = (resolution(float3(1.0f, 1.0f, 1.0f), memory)/4u)*4u; // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
		//LBM lbm(1u*lbm_N.x, 1u*lbm_N.y, 1u*lbm_N.z, 1u, 1u, 1u, 1.0f); // 1 GPU
		//LBM lbm(2u*lbm_N.x, 1u*lbm_N.y, 1u*lbm_N.z, 2u, 1u, 1u, 1.0f); // 2 GPUs
		//LBM lbm(2u*lbm_N.x, 2u*lbm_N.y, 1u*lbm_N.z, 2u, 2u, 1u, 1.0f); // 4 GPUs
		//LBM lbm(2u*lbm_N.x, 2u*lbm_N.y, 2u*lbm_N.z, 2u, 2u, 2u, 1.0f); // 8 GPUs

		// #########################################################################################################################################################################################
		for(uint i=0u; i<1000u; i++) {
			lbm.run(10u, 1000u*10u);
			mlups = max(mlups, to_uint((double)lbm.get_N()*1E-6/info.runtime_lbm_timestep_smooth));
		}
	} // make lbm object go out of scope to free its memory
	print_info("Peak MLUPs/s = "+to_string(mlups));
#if defined(_WIN32)
	wait();
#endif // Windows
} /**/
#endif // BENCHMARK

//-------------Este es para pruebas simples. Basado en Aerodynamics of a cow
/*void main_setup() { // Drone drag force study; required extensions in defines.hpp: FP16S, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 4.0f, 2.0f), 4000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	//const float lbm_Re = 10000000.0f; //Reynolds number, high number means high turbulence. Don't really need it if i use nu

	const float si_u = 5.0f; //Flowspeed, [m/s], SI
	const float si_length = 0.831426f; //Characteristic length of the drone, [m], SI
	const float si_T = 1.5f; //Simulation time, [s], SI
	const float si_nu = 0.0000010023f; //Kinematic viscocity, [m²/s], SI
	const float si_rho = 1000.f; //Air density, [kg/m³], SI
	
	const float lbm_length = 1.0f*(float)lbm_N.x; //Drone spans full X width of the box
	const float lbm_u = 0.075f; //Better to keep this as is

	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho); //Conversion table, so to speak
	const float lbm_nu = units.nu(si_nu); //Conversion, m²/s to LBM units
	const ulong lbm_T = units.t(si_T); // Conversion, s to time steps
	print_info("Re = "+to_string(to_uint(units.si_Re(si_length,si_u,si_nu)))); //Sanity check print info

	
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu); //Pass lbm_nu instead of nu_from_Re
	
	// ###################################################################################### define geometry ######################################################################################
	const float size = 1.f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.55f*size, lbm.center().z+0.05f*size);
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(0.0f))*float3x3(float3(0, 0, 1), radians(60.0f));
	Clock clock;
	lbm.voxelize_stl(get_exe_path()+"../stl/Extended_Props.STL", center, rotation, size, TYPE_S|TYPE_X); // https://www.thingiverse.com/thing:4975964/files
	println(print_time(clock.stop()));
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u; //Initial freestream velocity in Y for non-solid cells
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // Domain faces as equilibrium boundaries, all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	const float3 com = lbm.object_center_of_mass(TYPE_S|TYPE_X);
	char csv_speed_buf[32]; sprintf(csv_speed_buf, "%g", si_u);
	const string csv = get_exe_path()+"export/rb_state_"+replace(string(csv_speed_buf), ".", "_")+"m_s.csv";
	write_file(csv, "step,si_time,force_x,force_y,force_z,torque_x,torque_y,torque_z\n"); // forces and torques in SI units
#if defined(INTERACTIVE_GRAPHICS) || defined(INTERACTIVE_GRAPHICS_ASCII)
	lbm.write_status();
	lbm.run(); // interactive mode: rendering thread requires lbm.run() to control its lifetime
	lbm.write_status();
#elif defined(GRAPHICS) // batch graphics: write frames and log
	lbm.write_status();
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			lbm.graphics.set_camera_free(float3(-0.0f*(float)Nx, -0.25*(float)Ny, 1.5f*(float)Nz), -180.0f, 90.0f, 90.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/top/");
			lbm.graphics.set_camera_free(float3(-0.1f*(float)Nx, -0.25*(float)Ny, 1.0f*(float)Nz), -0.0f, 90.0f, 90.0f); // side
			lbm.graphics.write_frame(get_exe_path()+"export/lateral/");
		}
		lbm.run(1u, lbm_T);
		const float3 f = lbm.object_force(TYPE_S|TYPE_X);
		const float3 tq = lbm.object_torque(com, TYPE_S|TYPE_X);
		write_line(csv, to_string(lbm.get_t())+","+to_string(units.si_t(lbm.get_t()))+","+to_string(units.si_F(f.x))+","+to_string(units.si_F(f.y))+","+to_string(units.si_F(f.z))+","+to_string(units.si_M(tq.x))+","+to_string(units.si_M(tq.y))+","+to_string(units.si_M(tq.z))+"\n");
	}
	lbm.write_status();
#else // no graphics: log without rendering
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		lbm.run(1u, lbm_T);
		const float3 f = lbm.object_force(TYPE_S|TYPE_X);
		const float3 tq = lbm.object_torque(com, TYPE_S|TYPE_X);
		write_line(csv, to_string(lbm.get_t())+","+to_string(units.si_t(lbm.get_t()))+","+to_string(units.si_F(f.x))+","+to_string(units.si_F(f.y))+","+to_string(units.si_F(f.z))+","+to_string(units.si_M(tq.x))+","+to_string(units.si_M(tq.y))+","+to_string(units.si_M(tq.z))+"\n");
	}
	lbm.write_status();
#endif
} /**/
//----------------Este es el main para el barrido de fuerzas a distintas orientaciones.
void main_setup() { // Drone drag force sweep; required extensions in defines.hpp: FP16S, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ---- barrido esférico: leer elevación y azimut de la línea de comandos ----
    // uso:  ./FluidX3D <elev_deg> <azim_deg>
    // si no se pasan argumentos, usa 0,0 (cuerpo en orientación de referencia)
    float elev_deg = 0.0f, azim_deg = 0.0f;
    {
        std::ifstream fin(get_exe_path()+"sweep_angles.txt");
        if(fin.good()) { fin >> elev_deg >> azim_deg; }
        else print_info("AVISO: no se encontró sweep_angles.txt, usando 0,0");
    }
    // ---------------------------------------------------------------------------	
	const uint3 lbm_N = resolution(float3(1.0f, 4.0f, 2.0f), 4000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	//const float lbm_Re = 10000000.0f; //Reynolds number, high number means high turbulence. Don't really need it if i use nu

	const float si_u = 5.0f; //Flowspeed, [m/s], SI
	const float si_length = 0.831426f; //Characteristic length of the drone, [m], SI
	const float si_T = 1.5f; //Simulation time, [s], SI
	const float si_nu = 0.0000010023f; //Kinematic viscocity, [m²/s], SI
	const float si_rho = 1000.f; //Air density, [kg/m³], SI
	
	const float lbm_length = 1.0f*(float)lbm_N.x; //Drone spans full X width of the box
	const float lbm_u = 0.075f; //Better to keep this as is

	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho); //Conversion table, so to speak
	const float lbm_nu = units.nu(si_nu); //Conversion, m²/s to LBM units
	const ulong lbm_T = units.t(si_T); // Conversion, s to time steps
	print_info("Re = "+to_string(to_uint(units.si_Re(si_length,si_u,si_nu)))); //Sanity check print info

	
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu); //Pass lbm_nu instead of nu_from_Re
	
	// ###################################################################################### define geometry ######################################################################################
	const float size = 1.f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.55f*size, lbm.center().z+0.05f*size);
	// Rotación compuesta que apunta el cuerpo a la dirección (elev, azim) sobre la
	// esfera, relativa al flujo fijo en +Y.
	//   azimut: giro alrededor de Y (eje del flujo)  -> barre izquierda/derecha
	//   elevación: giro alrededor de X               -> barre arriba/abajo (proa/popa)
	// El orden importa: primero elevación en el marco del cuerpo, luego azimut.
	const float3x3 rotation =
		float3x3(float3(0, 1, 0), radians(azim_deg))   // azimut alrededor de Y (flujo)
		* float3x3(float3(1, 0, 0), radians(elev_deg));  // elevación alrededor de X
	Clock clock;
	lbm.voxelize_stl(get_exe_path()+"../stl/Extended_Props.STL", center, rotation, size, TYPE_S|TYPE_X); // https://www.thingiverse.com/thing:4975964/files
	println(print_time(clock.stop()));
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u; //Initial freestream velocity in Y for non-solid cells
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // Domain faces as equilibrium boundaries, all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	const float3 com = lbm.object_center_of_mass(TYPE_S|TYPE_X);
	// nombre del CSV incluye la orientación, para no sobrescribir entre casos
	char tag[64];
	sprintf(tag, "e%+03d_a%+03d", (int)lround(elev_deg), (int)lround(azim_deg));
	const string csv = get_exe_path()+"export/rb_state_"+string(tag)+".csv";
	write_file(csv, "step,si_time,force_x,force_y,force_z,torque_x,torque_y,torque_z\n"); // forces and torques in SI units
#if defined(INTERACTIVE_GRAPHICS) || defined(INTERACTIVE_GRAPHICS_ASCII)
	lbm.write_status();
	lbm.run(); // interactive mode: rendering thread requires lbm.run() to control its lifetime
	lbm.write_status();
#elif defined(GRAPHICS) // batch graphics: write frames and log
	lbm.write_status();
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			//lbm.graphics.set_camera_free(float3(-0.0f*(float)Nx, -0.25*(float)Ny, 1.5f*(float)Nz), -180.0f, 90.0f, 90.0f);
			//lbm.graphics.write_frame(get_exe_path()+"export/top/");
			//lbm.graphics.set_camera_free(float3(-0.1f*(float)Nx, -0.25*(float)Ny, 1.0f*(float)Nz), -0.0f, 90.0f, 90.0f); // side
			//lbm.graphics.write_frame(get_exe_path()+"export/lateral/");
		}
		lbm.run(1u, lbm_T);
		const float3 f = lbm.object_force(TYPE_S|TYPE_X);
		const float3 tq = lbm.object_torque(com, TYPE_S|TYPE_X);
		write_line(csv, to_string(lbm.get_t())+","+to_string(units.si_t(lbm.get_t()))+","+to_string(units.si_F(f.x))+","+to_string(units.si_F(f.y))+","+to_string(units.si_F(f.z))+","+to_string(units.si_M(tq.x))+","+to_string(units.si_M(tq.y))+","+to_string(units.si_M(tq.z))+"\n");
	}
	lbm.write_status();
#else // no graphics: log without rendering
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		lbm.run(1u, lbm_T);
		const float3 f = lbm.object_force(TYPE_S|TYPE_X);
		const float3 tq = lbm.object_torque(com, TYPE_S|TYPE_X);
		write_line(csv, to_string(lbm.get_t())+","+to_string(units.si_t(lbm.get_t()))+","+to_string(units.si_F(f.x))+","+to_string(units.si_F(f.y))+","+to_string(units.si_F(f.z))+","+to_string(units.si_M(tq.x))+","+to_string(units.si_M(tq.y))+","+to_string(units.si_M(tq.z))+"\n");
	}
	lbm.write_status();
#endif
} /**/

//-----------Utiliza este main_setup para probar tu orientación de malla con renderizado, o para casos simples sin movimiento rígido ni fuerzas de volumen. Para casos con movimiento rígido, fuerzas de volumen. Recuerda activar las extensiones necesarias en defines.hpp según el caso.
//-----------La ejecución es como sigue: ./make.sh a b, donde a y b son argumentos que representan la elevación y el azimut de la orientación que quieres probar, respectivamente. Si no se pasan argumentos, se asume 0,0 (orientación de referencia). El resultado es una imagen renderizada desde arriba
//-----------Revisa algunos casos de prueba (0,0), (90,0), (0,90), (45,45) para verificar que la convención de rotación es la que esperas.
//FP16s, FORCE_FIELD,EQUILIBRIUM_BOUNDARIES, SUBGRID, GRAPHICS
/*void main_setup() { // Orientation check: render top view, 1 step
	// ---- leer elevación y azimut de la línea de comandos ----
	float elev_deg = 0.0f, azim_deg = 0.0f;
	if(main_arguments.size() >= 1u) elev_deg = to_float(main_arguments[0]);
	if(main_arguments.size() >= 2u) azim_deg = to_float(main_arguments[1]);

	// ---- mismo dominio y unidades que el caso de producción ----
	const uint3 lbm_N = resolution(float3(1.0f, 4.0f, 2.0f), 4000u);
	const float si_u = 5.0f;
	const float si_length = 0.831426f;
	const float si_T = 1.5f;
	const float si_nu = 0.0000010023f;
	const float si_rho = 1000.f;
	const float lbm_length = 1.0f*(float)lbm_N.x;
	const float lbm_u = 0.075f;
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);

	// ---- geometría con rotación parametrizada (la que queremos verificar) ----
	const float size = 1.f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.55f*size, lbm.center().z+0.05f*size);
	// Rotación candidata: azimut alrededor de Y (eje del flujo), elevación alrededor de X.
	// >>> ESTA es la convención que estamos verificando visualmente <<<
	const float3x3 rotation =
	      float3x3(float3(0, 1, 0), radians(azim_deg))
	    * float3x3(float3(1, 0, 0), radians(elev_deg));
	lbm.voxelize_stl(get_exe_path()+"../stl/Extended_Props.STL", center, rotation, size, TYPE_S|TYPE_X);

	// ---- freestream + fronteras (igual que producción) ----
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();
	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E;
	});

	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE;

	// ---- correr UN paso y renderizar solo la vista superior ----
	lbm.run(0u, 1u);   // inicializa
	lbm.run(1u, 1u);   // un paso

	// carpeta por orientación (write_frame solo acepta ruta de carpeta en tu
	// versión; el PNG sale con nombre automático tipo image-000001.png dentro)
	char tag[64];
	sprintf(tag, "e%+03d_a%+03d", (int)lround(elev_deg), (int)lround(azim_deg));
	const string outdir = get_exe_path()+"export/orient_check/"+string(tag)+"/";

	// cámara superior (reusada de tu caso de producción, línea 695)
	lbm.graphics.set_camera_free(float3(0.0f*(float)Nx, -0.25*(float)Ny, 1.5f*(float)Nz), -180.0f, 90.0f, 90.0f);
	lbm.graphics.write_frame(outdir);

	print_info("orientacion renderizada: elev="+to_string(elev_deg)+" azim="+to_string(azim_deg)+" -> "+string(tag)+"/");
} /**/
