package main

import (
	"bufio"
	"fmt"
	"io/fs"
	"os"
	"strconv"
)

// this function gets the path to a directory from the user
func get_path() string {
	var path string

	fmt.Println("-------------------------------------------------------------------------------")
	fmt.Println("Enter the path to the desired directory:")
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
			return path
		}
	}
}

// this function opens a directory and returns an array containing the info on the files in that directory
func open_directory(path string) []fs.DirEntry {
	fmt.Println("system: \x1b[34mOpening directory...\x1b[0m")
	fmt.Println("-------------------------------------------------------------------------------")
	files, err := os.ReadDir(path)
	if err != nil {
		fmt.Print("system: \x1b[31mERROR - ", err)
		fmt.Println("\x1b[0m")
		fmt.Println("-------------------------------------------------------------------------------")
		os.Exit(1)
	}
	fmt.Println("system: \x1b[32mSuccessfully opened directory\x1b[0m")
	fmt.Println("-------------------------------------------------------------------------------")
	return files
}

// this function checks if a path to a folder is valid
func path_is_valid(path string) bool {
	_, err := os.Stat(path)
	if err == nil {
		fmt.Print("system: \x1b[32mSuccessfully found directory\x1b[0m\n")
		fmt.Println("-------------------------------------------------------------------------------")
		return true
	}
	if os.IsNotExist(err) {
		fmt.Print("system: \x1b[31mERROR - ", err)
		fmt.Println("\x1b[0m")
		fmt.Println("-------------------------------------------------------------------------------")
		return false
	}
	return false
}

func main() {

	trauma_head := 0
	trauma_torso := 0
	trauma_lower_ext := 0
	amputation_lower_ext := 0
	trauma_upper_ext := 0
	amputation_upper_ext := 0
	severe_hemorrhage := 0

	path := get_path()
	directory := open_directory(path)
	for _, current_file := range directory {
		// opening file
		open_file, err := os.Open(path + "/" + current_file.Name())
		if err != nil {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------------------")
			os.Exit(1)
		}

		fileScanner := bufio.NewScanner(open_file)
		fileScanner.Split(bufio.ScanLines)
		var file_lines []string

		for fileScanner.Scan() {
			file_lines = append(file_lines, fileScanner.Text())
		}

		for _, line := range file_lines {
			affliction_type, err := strconv.Atoi(string(line[0]))
			if err != nil {
				fmt.Print("system: \x1b[31mERROR - ", err)
				fmt.Println("\x1b[0m")
				fmt.Println("-------------------------------------------------------------------------------")
				os.Exit(1)
			}
			switch affliction_type {
			case 0:
				trauma_head++
			case 1:
				trauma_torso++
			case 2:
				trauma_lower_ext++
			case 3:
				amputation_lower_ext++
			case 4:
				trauma_upper_ext++
			case 5:
				amputation_upper_ext++
			case 6:
				severe_hemorrhage++
			default:
				fmt.Println("Invalid affliction type")
				os.Exit(1)
			}
		}

		// closing file
		err = open_file.Close()
		if err != nil {
			fmt.Print("system: \x1b[31mERROR - ", err)
			fmt.Println("\x1b[0m")
			fmt.Println("-------------------------------------------------------------------------------")
			os.Exit(1)
		}
	}

	fmt.Println("+---------------------------+---------+")
	fmt.Printf("| %-25s | %7s |\n", "Class", "Count")
	fmt.Println("+---------------------------+---------+")
	fmt.Printf("| %-25s | %7d |\n", "TRAUMA HEAD", trauma_head)
	fmt.Println("+---------------------------+---------+")
	fmt.Printf("| %-25s | %7d |\n", "TRAUMA TORSO", trauma_torso)
	fmt.Println("+---------------------------+---------+")
	fmt.Printf("| %-25s | %7d |\n", "TRAUMA LOWER EXT", trauma_lower_ext)
	fmt.Println("+---------------------------+---------+")
	fmt.Printf("| %-25s | %7d |\n", "AMPUTATION LOWER EXT", amputation_lower_ext)
	fmt.Println("+---------------------------+---------+")
	fmt.Printf("| %-25s | %7d |\n", "TRAUMA UPPER EXT", trauma_upper_ext)
	fmt.Println("+---------------------------+---------+")
	fmt.Printf("| %-25s | %7d |\n", "SEVERE HEMORRHAGE", severe_hemorrhage)
	fmt.Println("+---------------------------+---------+")
}
