
valgrind  --tool=memcheck --trace-children=yes --tool=drd --trace-alloc=yes --log-file=valgrind-%p.log spawn-fcgi -p 2000 -n -- $@
