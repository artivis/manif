# manif documentation

% Include content from [../README.md](../README.md)

```{include} ../README.md
    :start-after: <!-- Include start manif intro -->
    :end-before: <!-- Include stop manif intro -->
```

<!-- Including the links doesn't seem to work, -->
<!-- they arent resolved -->
<!-- % Include content from [../README.md](../README.md)
```{include} ../README.md
    :start-after:  Include start manif URLs
    :end-before:  Include stop manif URLs
``` -->

[barrau15]: https://arxiv.org/pdf/1410.1465.pdf
[fourmy19]: https://hal.science/hal-02183498/document
[kelly24]: https://arxiv.org/abs/2312.07555
[jsola18]: http://arxiv.org/abs/1812.01537
[jsola-iri-lecture]: https://www.youtube.com/watch?v=nHOcoIyJj2o
[cheat_sheet]: paper/Lie_theory_cheat_sheet.pdf
[IRI-UPC]: https://www.iri.upc.edu/

---

## In this documentation

````{grid} 1 1 2 2
```{grid-item-card} [Tutorials](tutorial/index)

**Start here** with an hands-on introduction to **manif** for new users, guiding you through installation and using manif in your project.
```

```{grid-item-card} [How-to guides](howto/index)

**Step-by-step guides** covering key operations and common tasks.
```
````

````{grid} 1 1 2 2
```{grid-item-card} [Reference](reference/index)

**Technical information**.
```
```{grid-item-card} [Explanation](explanation/index)

**Discussion and clarification** of key topics.
```
````

```{toctree}
:hidden:

tutorial/index
howto/index
reference/index
explanation/index
```
