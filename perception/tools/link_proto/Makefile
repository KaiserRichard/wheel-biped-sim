# CANDIDATE; PROPOSED host-only build. No firmware integration.
CC = cc
CFLAGS = -std=c99 -Wall -Wextra -Werror -pedantic -O2
.PHONY: test clean
tests/test_link_proto: link_proto.c link_proto.h tests/test_link_proto.c
	$(CC) $(CFLAGS) -I. link_proto.c tests/test_link_proto.c -o $@
test: tests/test_link_proto
	./tests/test_link_proto
	PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
clean:
	rm -f tests/test_link_proto tests/test_link_proto_sanitize
	rm -rf tests/test_link_proto.dSYM tests/test_link_proto_sanitize.dSYM
