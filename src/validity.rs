//! Use for checking whether the `resources` directory is valid
//! Only available under `server` feature
use std::{fs, any::type_name, marker::PhantomData, collections::HashMap};
use colored::Colorize;
use serde_json;
use serde::Deserialize;

use crate::{prelude::*, client};

// ---------------------------------------------------- Generalization of behavior ----------------------------------------------------

pub trait ValidityTest {
	type AutoFixT: AutoFix + Clone;
	fn condition_description(&self) -> String;// Example: "File some/resource/file.file exists"
	fn check(&self) -> Vec<ValidityTestResult<Self::AutoFixT>>;// Multiple problems are possible
}

pub trait AutoFix {
	fn description(&self) -> String;
	fn fix(&self) -> Result<(), String>;
	fn ui(&self) -> Result<(), String> {
		println!("\tAuto fix: {}", self.description());
		let answer = prompt("\tRun? [y/n]");
		match answer.to_lowercase().as_str() {
			"y" => self.fix(),
			_ => Ok(())
		}
	}
}

impl AutoFix for () {
	fn description(&self) -> String {
		"Placeholder AutoFix implementation for unit type".to_owned()
	}
	fn fix(&self) -> Result<(), String> {
		panic!("Attempt call fix() on placeholder AutoFix implementation for unit type");
	}
}

#[derive(Clone)]
pub enum ValidityTestResult<T: Clone> {
	Ok,
	Problem {
		type_: ProblemType,
		message: String,
		auto_fix_opt: Option<T>
	}
}

impl<T: Clone> ValidityTestResult<T> {
	pub fn problem(
		type_: ProblemType,
		message: String,
		auto_fix_opt: Option<T>
	) -> Self {
		Self::Problem {
			type_,
			message,
			auto_fix_opt
		}
	}
	pub fn is_ok(&self) -> bool {
		match self {
			Self::Ok => true,
			Self::Problem{..} => false
		}
	}
}

#[derive(Clone)]
pub enum ProblemType {
	Warning,
	Error
}

impl ProblemType {
	pub fn to_string(&self) -> String {
		match self {
			Self::Warning => "Warning".bright_yellow().to_string(),
			Self::Error => "Error".red().to_string()
		}
	}
}

// ---------------------------------------------------- Useful stuff ----------------------------------------------------

pub struct GenericValidityTest<T, E, F> where F: AutoFix + Clone{
	pub condition_description: String,
	pub result: Result<T, E>,
	pub fix_opt: Option<F>// AutoFix type
}

impl<T, E, F> GenericValidityTest<T, E, F> where E: ToString, F: AutoFix + Clone {
	pub fn new(
		condition_description: String,
		result: Result<T, E>,
		fix_opt: Option<F>
	) -> Self {
		Self {
			condition_description,
			result,
			fix_opt
		}
	}
}

impl<T, E, F> ValidityTest for GenericValidityTest<T, E, F> where E: ToString, F: AutoFix + Clone {
	type AutoFixT = F;
	fn condition_description(&self) -> String {
		self.condition_description.clone()
	}
	fn check(&self) -> Vec<ValidityTestResult<Self::AutoFixT>> {
		vec![
			match &self.result {
				Ok(_) => ValidityTestResult::Ok,
				Err(e) => ValidityTestResult::Problem{type_: ProblemType::Error, message: e.to_string(), auto_fix_opt: self.fix_opt.clone()}
			}
		]
	}
}

impl<T, E> ValidityTest for Result<T, E> where T: ValidityTest, E: ToString {
	type AutoFixT = T::AutoFixT;
	fn condition_description(&self) -> String {
		match &self {
			Ok(tester) => format!("This Result::Ok(_): {}", tester.condition_description()),
			Err(_) => "This Result<_, _> is Ok(_)".to_owned()
		}
	}
	fn check(&self) -> Vec<ValidityTestResult<Self::AutoFixT>> {
		match &self {
			Ok(tester) => tester.check(),
			Err(e) => vec![ValidityTestResult::Problem{type_: ProblemType::Error, message: e.to_string(), auto_fix_opt: None}]
		}
	}
}

pub struct ResourceExistanceChecker {
	path: String,
	file: bool
}

impl ResourceExistanceChecker {
	pub fn new(
		path: String,
		file: bool
	) -> Self {
		Self {
			path,
			file
		}
	}
}

impl ValidityTest for ResourceExistanceChecker {
	type AutoFixT = DirectoryExistanceAutoFix;
	fn condition_description(&self) -> String {
		let file_or_folder = match self.file{true => "File", false => "Directory"};
		format!("{} \"{}\" exists", file_or_folder, &self.path)
	}
	fn check(&self) -> Vec<ValidityTestResult<DirectoryExistanceAutoFix>> {
		let mut out = ValidityTestResult::Ok;
		if self.file {
			if fs::read_to_string(&self.path).is_err() {
				out = ValidityTestResult::Problem{
					type_: ProblemType::Error,
					message: format!("File doesn't exist at \"{}\"", &self.path),
					auto_fix_opt: None
				};
			}
		}
		else {
			if fs::read_dir(&self.path).is_err() {
				out = ValidityTestResult::Problem{
					type_: ProblemType::Error,
					message: format!("Directory doesn't exist at \"{}\"", &self.path),
					auto_fix_opt: Some(DirectoryExistanceAutoFix::new(self.path.clone()))
				};
			}
		}
		// Done
		vec![out]
	}
}

#[derive(Clone)]
pub struct DirectoryExistanceAutoFix {
	path: String
}

impl DirectoryExistanceAutoFix {
	pub fn new(path: String) -> Self {
		Self {
			path
		}
	}
}

impl AutoFix for DirectoryExistanceAutoFix {
	fn description(&self) -> String {
		format!("Create directory \"{}\"", &self.path)
	}
	fn fix(&self) -> Result<(), String> {
		to_string_err(fs::create_dir_all(&self.path))
	}
}

pub struct ResourceDeserializationChecker<T: for<'a> Deserialize<'a>> {
	path: String,
	phantom_data: PhantomData<T>// So that T can be "used"
}

impl<T: for<'a> Deserialize<'a>> ResourceDeserializationChecker<T> {
	pub fn new(path: String) -> Self {
		Self {
			path,
			phantom_data: PhantomData
		}
	}
}

impl<T: for<'a> Deserialize<'a>> ValidityTest for ResourceDeserializationChecker<T> {
	type AutoFixT = ();
	fn condition_description(&self) -> String {
		format!("the JSON file at \"{}\" is deserializable as {}", &self.path, type_name::<T>())
	}
	fn check(&self) -> Vec<ValidityTestResult<Self::AutoFixT>> {
		// First, check that file exists
		let res = &ResourceExistanceChecker::new(self.path.clone(), true).check()[0];
		let mut out = Vec::new();
		out.push(match res {
			ValidityTestResult::Ok => {
				// Check that it can be deserialized
				let deserialize_result: Result<T, serde_json::error::Error> = serde_json::from_str(&fs::read_to_string(&self.path).expect(&format!("Resource existence checker said \"{}\" cpuld be loaded, but this time it failed", &self.path)));
				match deserialize_result {
					Ok(_) => {
						ValidityTestResult::Ok
					},
					Err(e) => ValidityTestResult::problem(
						validity::ProblemType::Error,
						format!("Could not deserialize JSON file at \"{}\" as {}, error: {}", &self.path, type_name::<T>(), e.to_string()),
						None
					)
				}
			}
			ValidityTestResult::Problem{type_, message, ..} => ValidityTestResult::problem(type_.clone(), message.clone(), None)
		});
		// Done
		out
	}
}

/// Returns: Whether all errors have been resolved (DOESN'T include warnings)
fn test_ui<T: ValidityTest>(tester: T) -> bool {
	// Print test description
	println!("Checking that {}", tester.condition_description());
	// Out
	let mut out = true;
	// Run test
	let results: Vec<ValidityTestResult<T::AutoFixT>> = tester.check();
	// Enumerate problems
	let mut problem_i = 0;
	for result in results {
		match result {
			ValidityTestResult::Ok => {},
			ValidityTestResult::Problem{type_, message, auto_fix_opt} => {
				println!("{}. {}: {}", problem_i, type_.to_string(), &message);
				match auto_fix_opt {
					Some(auto_fix) => {
						match auto_fix.ui() {
							Ok(_) => {},
							Err(e) => {
								println!("\tAuto fix error: {}", e);
								if let ProblemType::Error = type_ {
									out = false;
								}
							}
						}
					},
					None => {}
				}
				problem_i += 1;
			}
		}
	}
	// Done
	out
}
#[allow(unused)]
fn vec_test_ui<T: ValidityTest>(testers: Vec<T>) -> bool {
	let mut out = true;
	for tester in testers {
		if !test_ui(tester) {
			out = false;
		}
	}
	// Done
	out
}

pub fn all_tests() {
	// Resource dir
	test_ui(ResourceExistanceChecker::new(resource_interface::RESOURCES_DIR.to_owned(), false));
	// Paths
	test_ui(ResourceExistanceChecker::new(resource_interface::PATH_TYPES_DIR.to_owned(), false));
	for entry_res in fs::read_dir(resource_interface::PATH_TYPES_DIR).unwrap() {
		let entry = entry_res.unwrap();
		if entry.metadata().unwrap().is_file() {
			let path = entry.path().to_str().expect("Could not get &str from path").to_owned();
			test_ui(ResourceDeserializationChecker::<PathType>::new(path));
		}
	}
	// Server's users file
	test_ui(ResourceDeserializationChecker::<HashMap<String, String>>::new(resource_interface::USERS_FILE.to_owned()));
	// Client's login file
	test_ui(ResourceDeserializationChecker::<HashMap<String, String>>::new(resource_interface::CLIENT_LOGIN_FILE.to_owned()));
	// Client's hardware calibration file
	test_ui(ResourceDeserializationChecker::<client::hardware_controller::Calibration>::new(resource_interface::CALIBRATION_FILE.to_owned()));
	// Client's settings file
	test_ui(ResourceDeserializationChecker::<client::Settings>::new(resource_interface::CLIENT_SETTINGS_FILE.to_owned()));
	// Vehicles
	test_ui(ResourceExistanceChecker::new(resource_interface::VEHICLES_DIR.to_owned(), false));
	for entry_res in fs::read_dir(resource_interface::VEHICLES_DIR).unwrap() {
		let entry = entry_res.unwrap();
		if entry.metadata().unwrap().is_dir() {
			let path = entry.path().to_str().expect("Could not get &str from path").to_owned() + "/" + resource_interface::VEHICLE_STATIC_JSON_FILENAME;
			test_ui(ResourceDeserializationChecker::<VehicleStatic>::new(path.clone()));
			// Check that v_static.type_name == path filename
			// TODO: implement validity test for vehicle static
			/*let name: &str = entry.path().file_name().expect(&format!("Couldn't get filename for static vehicle path \"{}\"", &path)).to_str().unwrap();
			let de_type_name = resource_interface::load_static_vehicle(name).expect("This error thould have been detected earlier").type_name;
			test_ui(
				if de_type_name == *name {
					Ok(())
				}
				else {
					Err(format!("Deserialized static vehicle `type_name` field (\"{}\") does not match directory name (\"{}\")", de_type_name, name))
				}
			);*/
		}
	}
	// Maps
	test_ui(ResourceExistanceChecker::new(resource_interface::MAPS_DIR.to_owned(), false));
	for entry_res in fs::read_dir(resource_interface::MAPS_DIR).unwrap() {
		let entry = entry_res.unwrap();
		if entry.metadata().unwrap().is_dir() {
			let path = entry.path().to_str().expect("Could not get &str from path").to_owned() + "/metadata.json";
			test_ui(ResourceDeserializationChecker::<SaveMap>::new(path.clone()));
			test_ui(resource_interface::load_map_metadata(entry.path().file_name().expect(&format!("Couldn't get filename for map path \"{}\"", &path)).to_str().unwrap()));
			// TODO: map.generic.name == path filename
		}
	}
	// Worlds
	test_ui(ResourceExistanceChecker::new(resource_interface::WORLDS_DIR.to_owned(), false));
	for entry_res in fs::read_dir(resource_interface::WORLDS_DIR).unwrap() {
		let entry = entry_res.unwrap();
		if entry.metadata().unwrap().is_file() {
			let path = entry.path().to_str().expect("Could not get &str from entry PathBuf").to_owned();
			let filename_binding = entry.file_name();
			let filename = filename_binding.to_str().expect("Could not get &str from entry OsString");
			if filename == ".DS_Store" {
				continue;
			}
			// Test world load
			test_ui(GenericValidityTest::new(
				format!("World at {} can be loaded", &path),
				World::load(filename.strip_suffix(".json").expect(&format!("world filename \"{}\" didn't end with \".json\"", filename))),
				None::<()>
			));
		}
	}
}

pub fn main_ui() {
	println!("{} resource validation utility", APP_NAME);
	all_tests();
}