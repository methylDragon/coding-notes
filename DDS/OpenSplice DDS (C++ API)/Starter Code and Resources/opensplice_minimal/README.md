# opensplice_minimal

This package is a slightly edited version of: https://github.com/grassjelly/opensplice_minimal

You should use this only as a reference. For actual deployment, please use opensplice_boilerplate as starter code, since it simplifies a whole lot of stuff for you and removes a lot of boilerplate code that you will have to write!



## Running the demo:

1. Generate the message headers:

    They will go into /gen

    ```
    ./gencode
    ```

2. Build the package:

        ./compile

3. Run the built subscriber:

        ./build/sub

4. On another terminal, run the built publisher:

        ./build/pub