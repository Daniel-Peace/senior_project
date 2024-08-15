package main

// imported packages
import (
	"bufio"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
)

// stores errors
var err error

func main() {
	// declaring vars
	var directory string
	var pairs [][]int

	// clearing terminal
	clear_terminal()

	// getting user parameters
	directory, pairs = gets_user_input()

	// confirming with user
	confirm_parameters(directory, pairs)

	// replacing labels
	replace_labels(directory, pairs)
	fmt.Println("--------------------------------------------------------------------------------------")
}

// clears the terminal
func clear_terminal() {
	cmd := exec.Command("clear")
	cmd.Stdout = os.Stdout
	cmd.Run()
}

// gets user paramters
func gets_user_input() (string, [][]int) {
	// declaring vars
	var path string
	var pairs [][]int
	var list_of_pair_strings []string

	// prompting user
	fmt.Println("--------------------------------------------------------------------------------------")
	fmt.Println("Enter the path to the folder containing the labels to be reindexed")

	// creating scanner
	scanner := bufio.NewScanner(os.Stdin)

	// looping until valid path is entered
	for {
		// getting directory path from user
		fmt.Println("--------------------------------------------------------------------------------------")
		fmt.Print("\x1b[34m-> \x1b[0m")
		if scanner.Scan() {
			path = scanner.Text()
		} else {
			err = scanner.Err()
			fmt.Println("-------------------------------------------------------------------")
			print_error_message()
			error_shut_down()
		}

		if path_exists(path) {
			fmt.Println("-------------------------------------------------------------------")
			fmt.Print("\x1b[32mSuccessfully found directory\x1b[0m\n")
			break
		} else {
			fmt.Println("-------------------------------------------------------------------")
			print_error_message()
		}
	}

	// prompting user
	fmt.Println("--------------------------------------------------------------------------------------")
	fmt.Println("Enter your desired replacemnt pairs. ")
	fmt.Println("\tEx. \"14,1 15,2 16,3\" creates the following rules: ")
	fmt.Println("\t\t14 -> 1")
	fmt.Println("\t\t15 -> 2")
	fmt.Println("\t\t16 -> 3")

	for {
		// ensuring arrays or empty
		pairs = nil
		list_of_pair_strings = nil

		fmt.Println("--------------------------------------------------------------------------------------")
		fmt.Print("\x1b[34m-> \x1b[0m")

		// scanning and parsing user input
		if scanner.Scan() {
			// checking if the user entered an empty string
			if scanner.Text() == "" {
				fmt.Println("\x1b[31mERROR - No replacement pair provided\x1b[0m")
				continue
			}

			// splitting the user input into an array of string pairs
			list_of_pair_strings = strings.Split(scanner.Text(), " ")
		} else {
			err = scanner.Err()
			fmt.Println("--------------------------------------------------------------------------------------")
			print_error_message()
			error_shut_down()
		}

		// looping over replacement pairs to convert them to integers and append them to the array of pairs
		found_err := false
		for _, current_pair_string := range list_of_pair_strings {
			current_pair := strings.Split(current_pair_string, ",")

			// converting the first number to an integer
			var num1 int
			num1, err = strconv.Atoi(current_pair[0])
			if err != nil {
				fmt.Println("--------------------------------------------------------------------------------------")
				print_error_message()
				found_err = true
				break
			}

			// converting the second number to an integer
			var num2 int
			num2, err = strconv.Atoi(current_pair[1])
			if err != nil {
				fmt.Println("--------------------------------------------------------------------------------------")
				print_error_message()
				found_err = true
				break
			}

			// appending pairs to the array
			pairs = append(pairs, []int{num1, num2})
		}
		if found_err {
			continue
		}

		return path, pairs
	}
}

// confirms user-entered paramteres
func confirm_parameters(directory string, pairs [][]int) {
	// printing what the user has entered to confirm
	fmt.Println("--------------------------------------------------------------------------------------")
	fmt.Println("The path you entered was:")
	fmt.Printf("\t\x1b[35m%s\x1b[0m\n", directory)
	fmt.Println("--------------------------------------------------------------------------------------")
	fmt.Println("The replacement pairs you entered were:")
	for _, pair := range pairs {
		fmt.Printf("\t\x1b[35m%d -> %d\x1b[0m\n", pair[0], pair[1])
	}

	// looping until user has entered a valid option
	for {
		// prompting user
		fmt.Println("--------------------------------------------------------------------------------------")
		fmt.Println("Enter \x1b[32mp\x1b[0m to proceed or \x1b[31mc\x1b[0m to cancel:")
		fmt.Println("--------------------------------------------------------------------------------------")
		fmt.Print("-> ")

		// creating scanner
		scanner := bufio.NewScanner(os.Stdin)

		// getting user input
		var option string
		if scanner.Scan() {
			option = scanner.Text()
		}

		// responding to selected option
		if option == "c" {
			fmt.Println("--------------------------------------------------------------------------------------")
			ok_shut_down()
		} else if option != "p" {
			fmt.Println("--------------------------------------------------------------------------------------")
			fmt.Println("\x1b[31mERROR - Invalid option\x1b[0m")
		} else {
			break
		}
	}
}

// replaces label indexes
func replace_labels(directory string, pairs [][]int) {
	// Open the directory
	files, err := os.ReadDir(directory)
	if err != nil {
		fmt.Println("--------------------------------------------------------------------------------------")
		print_error_message()
		fmt.Println("--------------------------------------------------------------------------------------")
		error_shut_down()
	}

	// looping through files
	for _, file := range files {
		fmt.Println("--------------------------------------------------------------------------------------")
		// skipping class file
		if file.Name() == "classes.txt" {
			fmt.Println("Skipping class file")
			continue
		}

		// creating filepath from file name and directory path
		filePath := filepath.Join(directory, file.Name())

		// checking if the file is a txt
		if file.Name()[len(file.Name())-3:] != "txt" {
			continue
		}

		// opening file
		file_descriptor, err := os.OpenFile(string(filePath), os.O_RDWR, 0644)
		if err != nil {
			print_error_message()
			fmt.Println("--------------------------------------------------------------------------------------")
			error_shut_down()
		}
		fmt.Printf("Replacing labels in file \"\x1b[35m%s\x1b[0m\"...\n", file.Name())

		// creating scanner to read file
		file_scanner := bufio.NewScanner(file_descriptor)

		// buffer to hold contents to be written to the file
		var content_buffer strings.Builder

		// looping over file to find label indexes and swap replace as needed
		for file_scanner.Scan() {
			// getting contents of the current line in the file
			current_line := file_scanner.Text()

			// getting the first element of the line and converting it to an integer
			label_number, err := strconv.Atoi(strings.Split(current_line, " ")[0])
			if err != nil {
				print_error_message()
				fmt.Println("--------------------------------------------------------------------------------------")
				error_shut_down()
			}

			not_replaced := true
			for _, pair := range pairs {
				// checking if label needs to be replaced
				if label_number == pair[0] {
					// replacing label
					modified_line := strings.Replace(current_line, strconv.Itoa(label_number), strconv.Itoa(pair[1]), 1)
					fmt.Printf(" - Replacing label \x1b[35m%d\x1b[0m with label \x1b[35m%d\x1b[0m\n", label_number, pair[1])

					// write to content_buffer
					content_buffer.WriteString(modified_line + "\n")

					// setting replaced flag
					not_replaced = false

					break
				}
			}

			if not_replaced {
				// write to content_buffer
				content_buffer.WriteString(current_line + "\n")
			}
		}
		// writing to file
		file_descriptor.Truncate(0) // Clear the file
		file_descriptor.Seek(0, 0)  // Reset the file pointer to the beginning
		bytes_written, err := file_descriptor.WriteString(content_buffer.String())
		if err != nil {
			print_error_message()
			fmt.Println("--------------------------------------------------------------------------------------")
			error_shut_down()
		}
		fmt.Printf(" - wrote \x1b[35m%d\x1b[0m bytes to file \"\x1b[35m%s\x1b[0m\"\n", bytes_written, file.Name())

		file_descriptor.Close()
	}
}

// exits program with error
func error_shut_down() {
	fmt.Println("Shutting down...")
	fmt.Println("--------------------------------------------------------------------------------------")
	os.Exit(1)
}

// exits program with no error
func ok_shut_down() {
	fmt.Println("Shutting down...")
	fmt.Println("--------------------------------------------------------------------------------------")
	os.Exit(0)
}

// checks if the provided path exists
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

// prints error message in red text using the global error variable
func print_error_message() {
	fmt.Print("\x1b[31mERROR - ", err)
	fmt.Println("\x1b[0m")
	err = nil
}
