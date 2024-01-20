// Use for checking whether the `resources` directory is valid
// Only available under `server` feature
use std::fs;

use crate::prelude::*;

pub trait ValidityTest {
	type T: AutoFix;
	fn condition_description(&self) -> String;// Example: "File some/resource/file.file exists"
	fn check(&self) -> Vec<ValidityTestResult<Self::T>>;// Multiple problems are possible
}

pub trait AutoFix {
	fn description(&self) -> String;
	fn fix(&self) -> Result<(), String>;
}

pub enum ValidityTestResult<T> {
	Ok,
	Problem {
		type_: ProblemType,
		message: String,
		auto_fix_opt: Option<T>
	}
}

pub enum ProblemType {
	Warning,
	Error
}

impl ProblemType {
	pub fn to_string(&self) -> String {
		match self {
			Self::Warning => "Warning".to_owned(),
			Self::Error => "Error".to_owned()
		}
	}
}

pub struct ResourceExistanceChecker {
	path: String,
	file: bool
}

impl ValidityTest for ResourceExistanceChecker {
	type T = ResourceExistanceAutoFix;
	fn condition_description(&self) -> String {
		let file_or_folder = match self.file{true => "File", false => "Directory"};
		format!("{} \"{}\" exists", file_or_folder, &self.path)
	}
	fn check(&self) -> Vec<ValidityTestResult<ResourceExistanceAutoFix>> {
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
					auto_fix_opt: Some(ResourceExistanceAutoFix::new(self.path.clone()))
				};
			}
		}
		// Done
		vec![out]
	}
}

pub struct ResourceExistanceAutoFix {
	path: String
}

impl ResourceExistanceAutoFix {
	pub fn new(path: String) -> Self {
		Self {
			path
		}
	}
}

impl AutoFix for ResourceExistanceAutoFix {
	fn description(&self) -> String {
		format!("Create directory \"{}\"", &self.path)
	}
	fn fix(&self) -> Result<(), String> {
		to_string_err(fs::create_dir_all(&self.path))
	}
}

fn test_ui<T: ValidityTest>(tester: T) {
	// Print test description
	println!("{}", tester.condition_description());
	// Run test
	let results: Vec<ValidityTestResult<T::T>> = tester.check();
	// Enumerate problems
	let mut problem_i = 0;
	for result in results {
		match result {
			ValidityTestResult::Ok => {},
			ValidityTestResult::Problem{type_, message, auto_fix_opt} => {
				println!("{}. {}: {}", problem_i, type_.to_string(), &message);
				match auto_fix_opt {
					Some(auto_fix) => println!("	Auto fix: {}", auto_fix.description()),
					None => {}
				}
				problem_i += 1;
			}
		}
	}
}

pub fn main_ui() {
	println!("{} resource validation utility", APP_NAME);
	// TODO
	let tests: Vec<ResourceExistanceChecker> = vec![
		ResourceExistanceChecker{path: "some/random/path".to_owned(), file: false},
		ResourceExistanceChecker{path: "some/random/file.file".to_owned(), file: true}
	];
	for test in tests {
		test_ui(test);
	}
}