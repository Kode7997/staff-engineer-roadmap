
/*
This file explain unique_ptr and it's capabailities and use-cases

- One owner access memory at a time. when owner destroyed memory is freed.
- Give exclusive ownership to a pointer no other can access reserved memory
- No COPY allowed
- Move is allowed to transfer the ownership
- Overhead of deleting the resource is eased. As destructor is called automcatically which eventually solves MEM LEAK problem
- Avoids multiple returns in 

*/