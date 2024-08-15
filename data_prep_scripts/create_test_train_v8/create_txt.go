package main

import (
	"fmt"
	"io"
	"os"
	"os/exec"
)

// variables
var (
	input_dir_path           string
	output_dir_path          string
	train_images_folder_path string
	train_labels_folder_path string
	test_images_folder_path  string
	test_labels_folder_path  string
	err                      error
)

func main() {
	// prompting user
	clear_terminal()
	fmt.Println("-------------------------------------------------------------------")
	fmt.Println("Enter the directory path that contains your images and labels:")
	fmt.Println("-------------------------------------------------------------------")

	// looping until valid path is entered
	for {
		fmt.Print("-> ")
		_, err = fmt.Scan(&input_dir_path)
		if err != nil {
			fmt.Println("-------------------------------------------------------------------")
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------")
			return
		}
		fmt.Println("-------------------------------------------------------------------")

		if path_exists(input_dir_path) {
			fmt.Print("system: \x1b[32mSuccessfully found directory\x1b[0m\n")
			fmt.Println("-------------------------------------------------------------------")
			break
		} else {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------")
		}
	}

	// prompting user
	clear_terminal()
	fmt.Println("-------------------------------------------------------------------")
	fmt.Println("Enter the path to the directory which \ncontains train and test folders:")
	fmt.Println("-------------------------------------------------------------------")
	for {
		fmt.Print("-> ")
		_, err = fmt.Scan(&output_dir_path)
		if err != nil {
			fmt.Println("-------------------------------------------------------------------")
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------")
			return
		}

		fmt.Println("-------------------------------------------------------------------")
		fmt.Println("Checking if train/images folder exists...")
		fmt.Println("-------------------------------------------------------------------")

		train_images_folder_path = output_dir_path + "/train/images"

		if path_exists(train_images_folder_path) {
			fmt.Print("system: \x1b[32mSuccessfully found directory\x1b[0m\n")
			fmt.Println("-------------------------------------------------------------------")
		} else {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------")
		}

		fmt.Println("Checking if train/labels folder exists...")
		fmt.Println("-------------------------------------------------------------------")

		train_labels_folder_path = output_dir_path + "/train/labels"

		if path_exists(train_labels_folder_path) {
			fmt.Print("system: \x1b[32mSuccessfully found directory\x1b[0m\n")
			fmt.Println("-------------------------------------------------------------------")
		} else {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------")
		}

		fmt.Println("Checking if test/images folder exists...")
		fmt.Println("-------------------------------------------------------------------")

		test_images_folder_path = output_dir_path + "/test/images"

		if path_exists(train_images_folder_path) {
			fmt.Print("system: \x1b[32mSuccessfully found directory\x1b[0m\n")
			fmt.Println("-------------------------------------------------------------------")
		} else {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------")
		}

		fmt.Println("Checking if test/labels folder exists...")
		fmt.Println("-------------------------------------------------------------------")

		test_labels_folder_path = output_dir_path + "/test/labels"

		if path_exists(train_labels_folder_path) {
			fmt.Print("system: \x1b[32mSuccessfully found directory\x1b[0m\n")
			fmt.Println("-------------------------------------------------------------------")
			break
		} else {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------")
		}
	}

	// creating array of files from directory
	fmt.Println("system: \x1b[34mOpening directory...\x1b[0m")
	fmt.Println("-------------------------------------------------------------------")
	files, err := os.ReadDir(input_dir_path)
	if err != nil {
		fmt.Print("system: \x1b[31mERROR - ", err)
		fmt.Println("\x1b[0m")
		fmt.Println("-------------------------------------------------------------------")
		return
	} else {
		fmt.Println("system: \x1b[32mSuccessfully opened directory\x1b[0m")
		fmt.Println("-------------------------------------------------------------------")
	}

	// looping over files in directory
	file_counter := 0
	for _, file := range files {

		fmt.Printf("\tcurrent file: \x1b[35m%s\x1b[0m\n", file.Name())

		// opening current file
		fmt.Println("\t\x1b[34mopening file\x1b[0m")
		current_source_file, err := os.Open(input_dir_path + "/" + file.Name())
		if err != nil {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
		}

		if file_counter == 0 {

			if file.Name()[len(file.Name())-3:] == "png" {
				// creating empty duplicate of file
				fmt.Println("\t\x1b[34mcreating file\x1b[0m")
				current_dest_file, err := os.Create(test_images_folder_path + "/" + file.Name())
				if err != nil {
					fmt.Print("system: \x1b[31mERROR - ", err)
					fmt.Println("\x1b[0m")
				}

				// copying contents
				fmt.Println("\t\x1b[34mcopying file\x1b[0m")
				_, err = io.Copy(current_dest_file, current_source_file)
				if err != nil {
					fmt.Print("system: \x1b[31mERROR - ", err)
					fmt.Println("\x1b[0m")
				}

				current_dest_file.Close()
			}

			if file.Name()[len(file.Name())-3:] == "txt" {
				// creating empty duplicate of file
				current_dest_file, err := os.Create(test_labels_folder_path + "/" + file.Name())
				if err != nil {
					fmt.Print("system: \x1b[31mERROR - ", err)
					fmt.Println("\x1b[0m")
				}

				// copying contents
				_, err = io.Copy(current_dest_file, current_source_file)
				if err != nil {
					fmt.Print("system: \x1b[31mERROR - ", err)
					fmt.Println("\x1b[0m")
				}

				current_dest_file.Close()

				file_counter++
			}
		} else {

			if file.Name()[len(file.Name())-3:] == "png" {
				// creating empty duplicate of file
				current_dest_file, err := os.Create(train_images_folder_path + "/" + file.Name())
				if err != nil {
					fmt.Print("system: \x1b[31mERROR - ", err)
					fmt.Println("\x1b[0m")
				}

				// copying contents
				_, err = io.Copy(current_dest_file, current_source_file)
				if err != nil {
					fmt.Print("system: \x1b[31mERROR - ", err)
					fmt.Println("\x1b[0m")
				}

				current_dest_file.Close()
			}

			if file.Name()[len(file.Name())-3:] == "txt" {
				// creating empty duplicate of file
				current_dest_file, err := os.Create(train_labels_folder_path + "/" + file.Name())
				if err != nil {
					fmt.Print("system: \x1b[31mERROR - ", err)
					fmt.Println("\x1b[0m")
				}

				// copying contents
				_, err = io.Copy(current_dest_file, current_source_file)
				if err != nil {
					fmt.Print("system: \x1b[31mERROR - ", err)
					fmt.Println("\x1b[0m")
				}

				current_dest_file.Close()

				file_counter++
			}
		}
		current_source_file.Close()

		fmt.Println("-------------------------------------------------------------------")

		if file_counter == 5 {
			file_counter = 0
		}
	}
}

func clear_terminal() {
	cmd := exec.Command("clear")
	cmd.Stdout = os.Stdout
	cmd.Run()
}

func path_exists(path string) bool {
	_, err = os.Stat(path)
	if err == nil {
		return true
	}
	if os.IsNotExist(err) {
		return false
	}
	return false
}
