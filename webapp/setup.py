import setuptools

AUTHOR_USER_NAME = "Abderrazak Chahid"
AUTHOR_EMAIL = "abderrazak.chahid@gmail.com"
PROJECT_DESCRIPTION = "HAIS webserver"
setuptools.setup(
    author=AUTHOR_USER_NAME,
    author_email=AUTHOR_EMAIL,
    description=PROJECT_DESCRIPTION,
    package_dir={"": "src"},
    packages=setuptools.find_packages(where="src")
)
