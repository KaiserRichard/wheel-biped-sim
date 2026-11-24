CXX = c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -pedantic -pthread
PYTHON = python3

.PHONY: test clean
test: tests/test_recorder
	./tests/test_recorder
	PYTHONDONTWRITEBYTECODE=1 $(PYTHON) -m unittest discover -s tests -v

tests/test_recorder: tests/test_recorder.cpp event_recorder.h
	$(CXX) $(CXXFLAGS) tests/test_recorder.cpp -o $@

clean:
	rm -f tests/test_recorder
