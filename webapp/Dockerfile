FROM python:3.7

# copy the code files
WORKDIR /app
COPY . /app

# Build the environement
RUN pip install --upgrade pip
RUN pip install --no-cache-dir --upgrade -r requirements.txt

# # Code PyTesting
# RUN pip install pytest
# RUN pytest

# Run the software
EXPOSE 8080
# CMD ["ls", "-a"]
CMD ["uvicorn", "webapp:app", "--host", "0.0.0.0", "--port", "8080"]