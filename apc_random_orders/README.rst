.. image:: http://picknik.io/PickNik_Logo3.png
   :target: http://picknik.io/
   :alt: picknik logo


Generate Mock Amazon order
--------------------------

Simple Python utility to create a simulated bin inventory and random
json order, that you can get by running::

    python random_orders.py order.json

Note that you can repeat experiments setting the used seed, and modify
the likelyhood of the number of objects per bin too::

    usage: random_orders.py [-h] [--probabilites PROBABILITES] [--seed SEED]
                            filename

    positional arguments:
      filename              filename to save the json order to

    optional arguments:
      -h, --help            show this help message and exit
      --probabilites PROBABILITES, -p PROBABILITES
                            Quote delimited list of probabilites. Eg "[0.5, 0.2,
                            0.2, 0.1]". Determines the likelyhood of filling up
                            with [1, 2...] elements the bins that aren't
                            determined by the contest rules. Defaults to [0.7,
                            0.2, 0.1], so that there are no bins with more than 3
                            elements.
      --seed SEED, -s SEED
