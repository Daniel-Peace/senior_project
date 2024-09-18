package main

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
)

const ZEROS = "00000"

func main() {
	clear_terminal()
	path := get_path()
	name := get_name()
	index := get_index()
	rename_data(path, name, index)
}

func get_path() string {
	var path string

	fmt.Println("-------------------------------------------------------------------------------")
	fmt.Println("Enter the directory path that contains your images and labels to be renamed:")
	fmt.Println("-------------------------------------------------------------------------------")
	for {
		fmt.Print("-> ")
		_, err := fmt.Scan(&path)
		if err != nil {
			fmt.Println("-------------------------------------------------------------------------------")
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------------------")
			os.Exit(1)
		}
		fmt.Println("-------------------------------------------------------------------------------")

		if path_is_valid(path) {
			fmt.Print("system: \x1b[32mSuccessfully found directory\x1b[0m\n")
			fmt.Println("-------------------------------------------------------------------------------")
			return path
		} else {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------------------")
		}
	}
}

func get_name() string {
	var name string
	fmt.Println("Enter the new name you would like to give your files:")
	fmt.Println("-------------------------------------------------------------------------------")
	fmt.Print("-> ")
	_, err := fmt.Scan(&name)
	if err != nil {
		fmt.Println("-------------------------------------------------------------------------------")
		fmt.Print("system: \x1b[31mERROR - ", err)
		fmt.Println("\x1b[0m")
		fmt.Println("-------------------------------------------------------------------------------")
		os.Exit(1)
	}
	fmt.Println("-------------------------------------------------------------------------------")

	return name
}

func get_index() int {
	var index_string string
	fmt.Println("Enter the postfix integer you would like to start at:")
	fmt.Println("-------------------------------------------------------------------------------")
	fmt.Print("-> ")
	_, err := fmt.Scan(&index_string)
	if err != nil {
		fmt.Println("-------------------------------------------------------------------------------")
		fmt.Print("system: \x1b[31mERROR - ", err)
		fmt.Println("\x1b[0m")
		fmt.Println("-------------------------------------------------------------------------------")
		os.Exit(1)
	}
	fmt.Println("-------------------------------------------------------------------------------")

	index, err := strconv.Atoi(index_string)
	if err != nil {
		fmt.Print("system: \x1b[31mERROR - ", err)
		fmt.Println("\x1b[0m")
		fmt.Println("-------------------------------------------------------------------------------")
		os.Exit(1)
	}
	return index
}

// this function checks if a path to a folder is valid
func path_is_valid(path string) bool {
	_, err := os.Stat(path)
	if err == nil {
		return true
	}
	if os.IsNotExist(err) {
		return false
	}
	return false
}

// this function clears the terminal
func clear_terminal() {
	cmd := exec.Command("clear")
	cmd.Stdout = os.Stdout
	cmd.Run()
}

// this function handles renaming all the files in the directory
func rename_data(path string, name string, img_index int) {
	// creating array of files from directory
	fmt.Println("system: \x1b[34mOpening directory...\x1b[0m")
	fmt.Println("-------------------------------------------------------------------------------")
	files, err := os.ReadDir(path)
	if err != nil {
		fmt.Print("system: \x1b[31mERROR - ", err)
		fmt.Println("\x1b[0m")
		fmt.Println("-------------------------------------------------------------------------------")
		os.Exit(1)
	} else {
		fmt.Println("system: \x1b[32mSuccessfully opened directory\x1b[0m")
		fmt.Println("-------------------------------------------------------------------------------")
	}

	increment_postfix := false
	zeros := ZEROS
	// looping over files in directory and renaming them
	for index, file := range files {
		oldName := filepath.Join(path, file.Name())
		ext := filepath.Ext(file.Name())
		//baseName := file.Name()[:len(file.Name())-len(ext)]

		if index%2 == 0 {
			if ext != ".jpg" {
				fmt.Println("system: \x1b[31mERROR - expected png but found", ext[1:])
				fmt.Println("\x1b[0m")
				fmt.Println("-------------------------------------------------------------------------------")
				os.Exit(1)
			}

			postfix := zeros[:len(zeros)-len(strconv.Itoa(img_index))] + strconv.Itoa(img_index)
			fmt.Println(postfix)
			newName := filepath.Join(path, name+"_"+postfix+ext)
			err := os.Rename(oldName, newName)
			if err != nil {
				fmt.Print("system: \x1b[31mERROR - ", err)
				fmt.Println("\x1b[0m")
				fmt.Println("-------------------------------------------------------------------------------")
				os.Exit(1)
			}
			fmt.Println("Renamed", oldName, "to", newName)

			increment_postfix = false
		} else {
			if ext != ".txt" {
				fmt.Println("system: \x1b[31mERROR - expected txt but found", ext[1:])
				fmt.Println("\x1b[0m")
				fmt.Println("-------------------------------------------------------------------------------")
				os.Exit(1)
			}

			postfix := zeros[:len(zeros)-len(strconv.Itoa(img_index))] + strconv.Itoa(img_index)
			fmt.Println(postfix)

			newName := filepath.Join(path, name+"_"+postfix+ext)
			err := os.Rename(oldName, newName)
			if err != nil {
				fmt.Print("system: \x1b[31mERROR - ", err)
				fmt.Println("\x1b[0m")
				fmt.Println("-------------------------------------------------------------------------------")
				os.Exit(1)
			}
			fmt.Println("Renamed", oldName, "to", newName)

			increment_postfix = true
		}

		if increment_postfix {
			img_index++
		}
	}
}
