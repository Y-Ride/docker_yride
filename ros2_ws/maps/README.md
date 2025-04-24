The maps contain large .pcd files. They are pushed upto github to be stored, but they don't need to be pulled. Unless you need to. 

### How to Not Pull Specific Folders
You can use git's `sparse checkout` functionality to specify which folders you want to (or don't want to) pull from the remote repo.

Do the following:
1. Enable sparse checkout
```bash
git config core.sparseCheckout true
```
2. Create the file to specify which files you want or don't want
```bash
nano .git/info/sparse-checkout
```
> Note: Spelling is important. If this file is misspelled, it won't work.

Add the following to the `sparse-checkout` file:
```txt
# Specify to include all folders
/*

# Specify which files to exclude
!large_folder/
!*.pcd # Large file extentions
```
3. Apply changes
> Warning: Commit any modifications before proceeding or else you will lose them!
```bash
git checkout HEAD
```

### How to Pull Everything After Initially Doing Sparse Checkout
1. Turn off `sparse-checkout`
```bash
git config core.sparseCheckout false
```
2. Apply changes
> Warning: Commit modifications before proceeding
```bash
git checkout HEAD
```
