# FullMetalFalcons Note:
### Switch to the `competition` branch to use this repository. The `master` branch contains only the base Road Runner code.
To switch to the `competition` branch:
* In Android Studio
  * Use the **git** menu and select **Branches**
  * Select **Remote** -> **origin** -> **competition**
  * Then click **Checkout**
* On the GitHub web page, select `competition` from the drop-down in the top left of the repository code page
* Using git CLI, type `git checkout competition`

This follows the FTC branching guidance [here](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/fork_and_clone_github_repository/Fork-and-Clone-From-GitHub.html).

### Updating to the latest Road Runner code
This follows the steps [here](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/fork_and_clone_github_repository/Fork-and-Clone-From-GitHub.html?highlight=fork#updating-the-sdk-to-the-latest-version) and assumes all commands are run using the git CLI from the root directory of your clone:

#### Add Road Runner as an upstream:

`git remote add upstream https://github.com/acmerobotics/road-runner-quickstart`

#### Update the master & competition branches with the lastest code

```bash
git checkout master
git fetch upstream
git merge upstream/master
git push origin master
git checkout competition
git merge master
```

# Road Runner Quickstart

Check out the [docs](https://rr.brott.dev/docs/v1-0/tuning/).

